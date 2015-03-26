import logging
import numpy
import openravepy
from .. import exceptions


class OWDTrajectoryStatus(object):
    def __init__(self, id, time_added):
        from concurrent.futures import Future
        from owd_msgs.msg import TrajInfo

        self.id = id
        self.time_added = time_added
        self.status = TrajInfo.state_pending
        self.future = Future()
        self.future.set_running_or_notify_cancel()


class OWDClient(object):
    logger = logging.getLogger('OWDClient')

    def __init__(self, robot, dof_indices, ns='',
                 skew_threshold=0.1, max_age=180., position_field='positions',
                 limits_action=openravepy.KinBody.CheckLimitsAction.Nothing):
        """
        Client library for interacting with an OWD instance.
        
        This class updates the joint angles of a robot from WAMState messages
        and executes OpenRAVE trajectories using the owd_ortraj plugin.

        @param robot OpenRAVE robot
        @param dof_indices DOF indices to update
        @param ns namespace that OWD is running in
        @param skew_threshold maximum clock skew permitted (in seconds)
        @param max_age maximum age of a trajectory (in seconds)
        @param position_field WAMState field used to update DOF values
        @param limits_action option used to update DOF values
        """

        import collections
        import rospy
        import threading
        from owd_msgs.msg import WAMState
        from owd_msgs.srv import AddOrTrajectory, DeleteTrajectory

        if not hasattr(WAMState, position_field):
            raise ValueError(
                'WAMState message does not have a field named "{:s}".'.format(
                    position_field)
            )

        self.robot = robot
        self.dof_indices = numpy.array(dof_indices, dtype=int)
        self.limits_action = limits_action

        self.lock = threading.Lock()
        self.ns = ns
        self.position_field = position_field
        self.skew_threshold = skew_threshold
        self.max_age = max_age

        self._pending_queue = collections.OrderedDict()
        self._done_queue = []
        self._wamstate_msg = None

        self._wamstate_sub = rospy.Subscriber(ns + '/wamstate',
                WAMState, self._wamstate_callback)
        self._add_or_trajectory_srv = rospy.ServiceProxy(
                ns + '/AddOrTrajectory', AddOrTrajectory)
        self._delete_trajectory_srv = rospy.ServiceProxy(
                ns + '/DeleteTrajectory', DeleteTrajectory)

    def is_idle(self):
        from owd_msgs.msg import WAMState

        with self.lock:
            # Assume that HERB is not idle if we've executed a trajectory
            # without receiving an updated WAMState message. This is critical
            # to avoid a race condition when calling is_idle() immediately
            # after execute_trajectory().
            if self._pending_queue:
                added_time = max(traj_status.time_added
                                 for traj_status in self._pending_queue)

                if self._wamstate_msg.header.stamp <= added_time:
                    return False

            # Otherwise, return the current state of the WAM.
            return self._wamstate_msg.state in [
                WAMState.state_free,
                WAMState.state_fixed,
                WAMState.state_inactive
            ]

    def execute_trajectory(self, traj, synchronize=False, wait_for_start=False,
                           cancel_on_stall=True, cancel_on_force=False,
                           cancel_on_tactile=False):
        """
        Execute an OpenRAVE trajectory on the WAM.
        
        This function requires the owd_ortraj plugin to be loaded in OWD. This
        function returns a concurrent.futures.Future boolean. The future is
        completed when the trajectory finishes executing (true) or is aborted
        early (false).
        
        @param traj timed OpenRAVE trajectory to execute
        @param synchronize enable synchronization between OWD instances
        @param wait_for_start queue the trajectory as "paused"
        @param cancel_on_stall cancel the trajectory on stall
        @param cancel_on_force cancel the trajectory on force/torque input
        @param cancel_on_tactile cancel the trajectory on tactile input
        @return future boolean that indicates success
        """

        import rospy
        from owd_msgs.srv import AddOrTrajectoryRequest

        request = AddOrTrajectoryRequest(
            traj = traj.serialize(0),
            xml_id = traj.GetXMLId(),
            synchronize = synchronize,
            options = 0,
        )

        if wait_for_start:
            request.options |= AddOrTrajectoryRequest.opt_WaitForStart
        if cancel_on_stall:
            request.options |= AddOrTrajectoryRequest.opt_CancelOnStall
        if cancel_on_force:
            request.options |= AddOrTrajectoryRequest.opt_CancelOnForceInput
        if cancel_on_tactile:
            request.options |= AddOrTrajectoryRequest.opt_CancelOnTactileInput
        if synchronize:
            request.options |= AddOrTrajectoryRequest.opt_Synchronize

        # Perform the service call.
        response = self._call_service(
            self._add_or_trajectory_srv, request,
            base_message='Adding trajectory failed: '
        )

        if not response.id:
            raise exceptions.TrajectoryExecutionFailed(
                'Trajectory execution failed: OWD returned an empty ID.'
            )

        # Add this trajectory ID to the list of trajectories that we're
        # monitoring. We use the time returned by OWD to avoid potential
        # synchronization problems.
        with self.lock:
            trajectory_status = OWDTrajectoryStatus(
                id=response.id,
                time_added=response.time_added,
            )
            self._pending_queue[response.id] = trajectory_status

        return trajectory_status.future

    def update(self):
        import numpy
        import rospy
        from owd_msgs.msg import TrajInfo

        now = rospy.Time.now()
        env = self.robot.GetEnv()

        with self.lock:
            if self._wamstate_msg is None:
                return

            dof_values_raw = getattr(self._wamstate_msg, self.position_field)
            dof_values = numpy.array(dof_values_raw, dtype=float)

            if len(dof_values) != len(self.dof_indices):
                raise exceptions.PrPyException(
                    'WAMState field "{:s}" contains incorrect number of dofs:'
                    ' got {:d}, expected {:d}.'.format(
                        self.position_field, len(dof_values), len(self.dof_indices)
                    )
                )

            # Check for stale WAMState messages. This can happen if: (1) OWD
            # dies and stops publishing data or (2) there is significant clock
            # skew.
            clock_skew = (now - self._wamstate_msg.header.stamp).to_sec()
            if clock_skew > self.skew_threshold:
                raise exceptions.PrPyException(
                    'Have not received a WAMState message in {:.3f} seconds.'
                    ' Is "{:s}/wamstate" still being published?'.format(
                        clock_skew, self.ns
                    )
                )
            # Check for WAMState messages from the future. This indicates
            # significant clock skew.
            elif self.skew_threshold is not None and clock_skew < -self.skew_threshold:
                raise exceptions.PrPyException(
                    'Detected clock skew of {:.3f} seconds with OWD. Is the'
                    ' local machine using the same NTP server as the'
                    ' robot?'.format(-clock_skew)
                )

            # Update the status of trajectories that are executing.
            all_trajectories  = [ self._wamstate_msg.prev_trajectory ]
            all_trajectories += self._wamstate_msg.trajectory_queue

            for traj_info in all_trajectories:
                traj_status = self._pending_queue.get(traj_info.id)

                if traj_status is not None:
                    do_remove = False

                    if traj_status.status != traj_info.status:
                        traj_status.status = traj_info.status

                        # Trajectory finishes. Return success.
                        if traj_info.status == TrajInfo.state_done:
                            traj_status.future.set_result(True)
                        # Trajectory aborted. 
                        elif traj_info.status == TrajInfo.state_aborted:
                            traj_status.future.set_result(False)

            # Prune the queue of pending trajectories. We can't do this
            # in-place while iterating over the dictionary, so we create a
            # temporary removal list.
            pending_removals = []

            for traj_status in self._pending_queue:
                traj_age = now - traj_status.time_added
                
                # This trajectory has terminated (probably due to a set_result
                # call in the above loop).
                if traj_status.future.done():
                    pending_removals.append(traj_status)

                # This trajectory handle hasn't been updated within the
                # timeout, so we'll garbage collect it.
                elif traj_age > self.max_age:
                    traj_status.future.set_exception(
                        exceptions.TrajectoryExecutionFailed(
                            'Did not receive status update for trajectory'
                            ' "{:s}" in {:.3f} seconds.'.format(
                                traj_status.id, traj_age)
                        )
                    )
                    logger.warning(
                        'Garbage collected stale trajectory (age = %.3f s)'
                        ' from the set of pending trajectories. Did OWD crash?',
                        traj_age
                    )
                    pending_removals.append(traj_status)

            for traj_status in pending_removals:
                del self._pending_queue[traj_status.id]

        # Update the robot's joint values in OpenRAVE. The environment must be
        # locked here.
        self.robot.SetDOFValues(dof_values, self.dof_indices, self.limits_action)

    def cancel_trajectories(self, ids):
        from owd_msgs.srv import DeleteTrajectoryRequest

        self._call_service(
            self._delete_trajectory_srv,
            DeleteTrajectoryRequest(ids=list(ids)),
            base_message='Deleting trajectory failed: '
        )

    @staticmethod
    def _call_service(srv, request, base_message=''):
        # Make the service call.
        try:
            response = srv(request)
        except rospy.ServiceException as e:
            raise exceptions.TrajectoryExecutionFailed(
                base_message + 'Failed calling service: ' + e.message
            )

        # Validate the response.
        if not response.ok and response.reason:
            raise exceptions.TrajectoryExecutionFailed(
                base_message + response.reason
            )
        elif not response.ok and not response.reason:
            raise exceptions.TrajectoryExecutionFailed(
                base_message + 'An unknown error has occurred.'
            )
        elif response.reason:
            logger.warning('Service returned success with message: %s',
                response.reason
            )

        return response

    def _wamstate_callback(self, wamstate_msg):
        with self.lock:
            if (self._wamstate_msg is not None and wamstate_msg.header.stamp
                    < self._wamstate_msg.header.stamp):
                raise exceptions.PrPyException(
                    'Detected jump backwards in WAMState timestamps:'
                    ' {:s} < {:s}.'.format(
                        wamstate_msg.header.stamp,
                        self._wamstate_msg.header.stamp
                    )
                )

            self._wamstate_msg = wamstate_msg
