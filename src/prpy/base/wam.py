#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging, numpy, openravepy
from collections import namedtuple
from manipulator import Manipulator
from prpy.clone import Clone
from .. import util
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
                 skew_threshold=0.1, max_age=180.,
                 position_field='target_positions',
                 limits_actions=openravepy.KinBody.CheckLimitsAction.Nothing):
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

        if not hasattr(wamstate_msg, position_field):
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
            dof_values_raw = getattr(self._wamstate_msg, self.position_field)
            dof_values = numpy.array(dof_values_raw, dtype=float)

            if len(dof_values) != len(self.dof_indices):
                raise exceptions.PrPyException(
                    'WAMState field "{:s}" contains incorrect number of dofs:'
                    ' got {:d}, expected{;d}.'.format(
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
            elif clock_skew < -self.skew_threshold:
                raise exceptions.PrPyException(
                    'Detected clock skew of {.3f} seconds with OWD. Is the'
                    ' local machine using the same NTP server as the'
                    ' robot?'.format(-clock_skew)
                )

            # Update the status of trajectories that are executing.
            all_trajectories  = [ wamstate_msg.prev_trajectory ]
            all_trajectories += wamstate_msg.trajectory_queue

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
        self.robot.SetDOFValues(dof_values, dof_indices, self.limits_action)

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
            if wamstate_msg.header.stamp < self._wamstate_msg.header.stamp:
                raise exceptions.PrPyException(
                    'Detected jump backwards in WAMState timestamps:'
                    ' {:s} < {:s}.'.format(
                        wamstate_msg.header.stamp,
                        self._wamstate_msg.header.stamp
                    )
                )

            self._wamstate_msg = wamstate_msg


class WAM(Manipulator):
    def __init__(self, sim, owd_namespace,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        self.iktype = iktype

        self.controller = self.GetRobot().AttachController(name=self.GetName(),
            args='OWDController {0:s} {1:s}'.format('prpy', owd_namespace),
            dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)

        # Load the IK database.
        robot = self.GetRobot()
        if iktype is not None:
            from openravepy.databases.inversekinematics import InverseKinematicsModel
            self.ikmodel = InverseKinematicsModel(robot=robot, manip=self, iktype=iktype)
            if not self.ikmodel.load():
                self.ikmodel.generate(iktype=iktype, precision=4,
                                      freeindices=[ self.GetIndices()[2] ])
                self.ikmodel.save()

        # Enable servo motions in simulation mode.
        if sim:
            from prpy.simulation import ServoSimulator

            self.servo_simulator = ServoSimulator(self, rate=20, watchdog_timeout=0.1)
        # Setup service calls for trajectory exeuction via OWD.
        else:
            import rospy
            from owd_msgs.msg import AddOrTrajectory

            self._add_or_trajectory = rospy.ServiceProxy(
                owd_namespace + '/AddOrTrajectory', AddOrTrajectory)

    def CloneBindings(self, parent):
        self.__init__(sim=True, owd_namespace=None, iktype=parent.iktype)

    def SetStiffness(manipulator, stiffness):
        """Set the WAM's stiffness.
        Stiffness 0 is gravity compensation and stiffness 1 is position
        control. Values between 0 and 1 are experimental.
        @param stiffness value between 0.0 and 1.0
        """
        if not (0 <= stiffness <= 1):
            raise Exception('Stiffness must in the range [0, 1]; got %f.' % stiffness)

        if not manipulator.simulated:
            manipulator.controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

    def Servo(manipulator, velocities):
        """Servo with a vector of instantaneous joint velocities.
        @param velocities joint velocities, in radians per second
        """
        num_dof = len(manipulator.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. '
                             'Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if not manipulator.simulated:
            manipulator.controller.SendCommand('Servo ' + ' '.join([ str(qdot) for qdot in velocities ]))
        else:
            manipulator.controller.Reset(0)
            manipulator.servo_simulator.SetVelocity(velocities)

    def ServoTo(manipulator, target, duration, timeStep=0.05, collisionChecking=True):
        """Servo the arm towards a target configuration over some duration.
        Servos the arm towards a target configuration with a constant joint
        velocity. This function uses the \ref Servo command to control the arm
        and must be called repeatidly until the arm reaches the goal. If \tt
        collisionChecking is enabled, then the servo will terminate and return
        False if a collision is detected with the simulation environment.
        @param target desired configuration
        @param duration duration in seconds
        @param timestep period of the control loop, in seconds
        @param collisionchecking check collisions in the simulation environment
        @return whether the servo was successful
        """
        steps = int(math.ceil(duration/timeStep))
        original_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
        velocity = numpy.array(target-manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices()))
        velocities = v/steps#[v/steps for v in velocity]
        inCollision = False 
        if collisionChecking:
            inCollision = manipulator.CollisionCheck(target)

        if not inCollision:
            for i in range(1,steps):
                manipulator.Servo(velocities)
                time.sleep(timeStep)
            manipulator.Servo([0] * len(manipulator.GetArmIndices()))
            new_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
            return True
        else:
            return False

    def GetVelocityLimits(self, openrave=True, owd=True):
        """Get the OpenRAVE and OWD joint velocity limits.
        This function checks both the OpenRAVE and OWD joint velocity limits.
        If they do not match, a warning is printed and the minimum value is
        returned.
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        @return list of velocity limits, in radians per second
        """
        # Update the OpenRAVE limits.
        if openrave:
            or_velocity_limits = Manipulator.GetVelocityLimits(self)
            if self.simulated or not owd:
                return or_velocity_limits

        # Update the OWD limits.
        if owd and not self.simulated:
            args  = [ 'GetSpeed' ]
            args_str = ' '.join(args)
            owd_speed_limits_all = self.controller.SendCommand(args_str)
            #if we get nothing back, e.g. if the arm isn't running, return openrave lims
            if owd_speed_limits_all is None:
                return or_velocity_limits

            owd_speed_limits = map(float, owd_speed_limits_all.split(','));
            #first 7 numbers are velocity limits
            owd_velocity_limits = numpy.array(owd_speed_limits[0:len(self.GetIndices())])

            diff_arr = numpy.subtract(or_velocity_limits, owd_velocity_limits)
            max_diff = max(abs(diff_arr))

            if max_diff > 0.01:
                # TODO: Change this to use the logging framework.
                print('GetVelocityLimits Error: openrave and owd limits very different')
                print('\tOpenrave limits:\t' + str(or_velocity_limits))
                print('\tOWD limits:\t\t' + str(owd_velocity_limits))

            return numpy.minimum(or_velocity_limits, owd_velocity_limits)

        return or_velocity_limits
        
    def SetVelocityLimits(self, velocity_limits, min_accel_time,
                          openrave=True, owd=True):
        """Change the OpenRAVE and OWD joint velocity limits.
        Joint velocities that exceed these limits will trigger a velocity fault.
        @param velocity_limits vector of joint velocity limits in radians per second
        @param min_accel_time minimum acceleration time used to compute acceleration limits
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        """
        # Update the OpenRAVE limits.
        if openrave:
            Manipulator.SetVelocityLimits(self, velocity_limits, min_accel_time)

        # Update the OWD limits.
        if owd and not self.simulated:
            args  = [ 'SetSpeed' ]
            args += [ str(min_accel_time) ]
            args += [ str(velocity) for velocity in velocity_limits ]
            args_str = ' '.join(args)
            self.controller.SendCommand(args_str)

    def GetTrajectoryStatus(manipulator):
        """Gets the status of the current (or previous) trajectory executed by OWD.
        @return status of the current (or previous) trajectory executed
        """
        if not manipulator.simulated:
            return manipulator.controller.SendCommand('GetStatus')
        else:
            if manipulator.controller.IsDone():
                return 'done'
            else:
                return 'active'

    def ClearTrajectoryStatus(manipulator):
        """Clears the current trajectory execution status.
        This resets the output of \ref GetTrajectoryStatus.
        """
        if not manipulator.simulated:
            manipulator.controller.SendCommand('ClearStatus')

    def ExecuteTrajectory(self, traj, timeout=None):
        cspec = traj.GetConfigurationSpecification()
        #cspec.FindGroup

        if self.simulated:
            is_success = self.controller.SetPath(traj)

            if not is_success:
                raise exceptions.TrajectoryExecutionFailed(
                    'Trajectory execution failed: An unknown error has'
                    ' occurred.')

            return util.WaitForControllers([ self.controller ], timeout=timeout)
        else:
            from owd_msgs.msg import AddTrajectoryRequest, AddOrTrajectoryRequest


    def MoveUntilTouch(manipulator, direction, distance, max_distance=float('+inf'),
                       max_force=5.0, max_torque=None, ignore_collisions=None, **kw_args):
        """Execute a straight move-until-touch action.
        This action stops when a sufficient force is is felt or the manipulator
        moves the maximum distance. The motion is considered successful if the
        end-effector moves at least distance. In simulation, a move-until-touch
        action proceeds until the end-effector collids with the environment.
        @param direction unit vector for the direction of motion in the world frame
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param max_force maximum force in Newtons
        @param max_torque maximum torque in Newton-Meters
        @param execute optionally execute the trajectory
        @param ignore_collisions collisions with these objects are ignored in simulation
        @param **kw_args planner parameters
        @return felt_force flag indicating whether we felt a force.
        """
        # TODO: Is ignore_collisions a list of names or KinBody pointers?
        if max_torque is None:
            max_torque = numpy.array([100.0, 100.0, 100.0 ])
        
        ignore_col_obj_oldstate = []
        if ignore_collisions is None:
            ignore_collisions = []

        for ignore_col_with in ignore_collisions:
            ignore_col_obj_oldstate.append(ignore_col_with.IsEnabled())
            ignore_col_with.Enable(False)

        with manipulator.GetRobot().GetEnv():
            manipulator.GetRobot().GetController().SimulationStep(0)

            # Compute the expected force direction in the hand frame.
            direction = numpy.array(direction)
            hand_pose = manipulator.GetEndEffectorTransform()
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            with manipulator.GetRobot():
                old_active_manipulator = manipulator.GetRobot().GetActiveManipulator()
                manipulator.SetActive()
                traj = manipulator.PlanToEndEffectorOffset(direction, distance, max_distance=max_distance,
                                                           execute=False, **kw_args)

	collided_with_obj = False
        try:
            if not manipulator.simulated:
                manipulator.hand.TareForceTorqueSensor()
                #manipulator.GetRobot().ExecuteTrajectory(traj, execute=True, retime=True, blend=False) 
                manipulator.GetRobot().ExecuteTrajectory(traj, execute=True, retime=True, blend=True, stop_on_ft=True, 
								force_direction=force_direction, force_magnitude=max_force, torque=max_torque)
                for (ignore_col_with, oldstate) in zip(ignore_collisions, ignore_col_obj_oldstate):
                    ignore_col_with.Enable(oldstate)
            else:
                traj = manipulator.GetRobot().BlendTrajectory(traj)
                traj = manipulator.GetRobot().RetimeTrajectory(traj, stop_on_ft=True, force_direction=force_direction,
                                                               force_magnitude=max_force, torque=max_torque)
                traj_duration = traj.GetDuration()
                delta_t = 0.01

                traj_config_spec = traj.GetConfigurationSpecification()
                traj_angle_group = traj_config_spec.GetGroupFromName('joint_values')
                path_config_spec = openravepy.ConfigurationSpecification()
#                path_config_spec.AddDeltaTimeGroup()
                path_config_spec.AddGroup(traj_angle_group.name, traj_angle_group.dof, '')
#                path_config_spec.AddDerivativeGroups(1, False);
#                path_config_spec.AddDerivativeGroups(2, False);
#                path_config_spec.AddGroup('owd_blend_radius', 1, 'next')
                path_config_spec.ResetGroupOffsets()

                new_traj = openravepy.RaveCreateTrajectory(manipulator.GetRobot().GetEnv(), '')
                new_traj.Init(path_config_spec)

                for (ignore_col_with, oldstate) in zip(ignore_collisions, ignore_col_obj_oldstate):
                    ignore_col_with.Enable(oldstate)
                
                with manipulator.GetRobot():
                    manipulator.SetActive()
                    waypoint_ind = 0
                    for t in numpy.arange(0, traj_duration, delta_t):
                        traj_sample = traj.Sample(t)

                        waypoint = traj_config_spec.ExtractJointValues(traj_sample, manipulator.GetRobot(), manipulator.GetArmIndices())
                        manipulator.SetDOFValues(waypoint)
                        #if manipulator.GetRobot().GetEnv().CheckCollision(manipulator.GetRobot()):
                        for body in manipulator.GetRobot().GetEnv().GetBodies():
                            if manipulator.GetRobot().GetEnv().CheckCollision(manipulator.GetRobot(), body):
                                collided_with_obj = True
                                break
                        if collided_with_obj:
                            break
                        else:
                            #waypoint = numpy.append(waypoint,t)
                            new_traj.Insert(int(waypoint_ind), waypoint, path_config_spec)
                            waypoint_ind += 1

                    #new_traj = manipulator.GetRobot().BlendTrajectory(new_traj)
                    #new_traj = manipulator.GetRobot().RetimeTrajectory(new_traj)
                manipulator.GetRobot().ExecuteTrajectory(new_traj, execute = True, retime=True, blend=True)

            return collided_with_obj
        # Trajectory is aborted by OWD because we felt a force.
        except exceptions.TrajectoryAborted:
            return True
