#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: Siddhartha Srinivasa <siddh@cs.cmu.edu>
# Authors: Michael Koval <mkoval@cs.cmu.edu>
# Authors: David Butterworth <dbworth@cmu.edu>
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

import logging
import numpy
import openravepy
from .base import BasePlanner, PlanningError, ClonedPlanningMethod, Tags
from .. import util
from ..collision import DefaultRobotCollisionChecker 
from enum import Enum
from openravepy import CollisionOptions, CollisionOptionsStateSaver

logger = logging.getLogger(__name__)


class TerminationError(PlanningError):
    def __init__(self):
        super(TerminationError, self).__init__('Terminated by callback.')


class TimeLimitError(PlanningError):
    def __init__(self):
        super(TimeLimitError, self).__init__('Reached time limit.')


class Status(Enum):
    '''
    CONTINUE - keep going
    TERMINATE - stop gracefully and output the CACHEd trajectory
    CACHE_AND_CONTINUE - save the current trajectory and CONTINUE.
                         return the saved trajectory if TERMINATEd.
    CACHE_AND_TERMINATE - save the current trajectory and TERMINATE
    '''
    TERMINATE = -1
    CACHE_AND_CONTINUE = 0
    CONTINUE = 1
    CACHE_AND_TERMINATE = 2

    @classmethod
    def DoesTerminate(cls, status):
        return status in [cls.TERMINATE, cls.CACHE_AND_TERMINATE]

    @classmethod
    def DoesCache(cls, status):
        return status in [cls.CACHE_AND_CONTINUE, cls.CACHE_AND_TERMINATE]


class VectorFieldPlanner(BasePlanner):
    def __init__(self, robot_collision_checker=DefaultRobotCollisionChecker):
        super(VectorFieldPlanner, self).__init__()
        self.robot_collision_checker = robot_collision_checker

    def __str__(self):
        return 'VectorFieldPlanner'

    @ClonedPlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, timelimit=5.0,
                              pose_error_tol=0.01,
                              integration_interval=10.0,
                              **kw_args):
        """
        Plan to an end effector pose by following a geodesic loss function
        in SE(3) via an optimized Jacobian.

        @param robot
        @param goal_pose desired end-effector pose
        @param timelimit time limit before giving up
        @param pose_error_tol in meters
        @param integration_interval The time interval to integrate over
        @return traj
        """
        manip = robot.GetActiveManipulator()

        def vf_geodesic():
            """
            Define a joint-space vector field, that moves along the
            geodesic (shortest path) from the start pose to the goal pose.
            """
            twist = util.GeodesicTwist(manip.GetEndEffectorTransform(),
                                       goal_pose)
            dqout, tout = util.ComputeJointVelocityFromTwist(
                robot, twist, joint_velocity_limits=numpy.PINF)

            # Go as fast as possible
            vlimits = robot.GetDOFVelocityLimits(robot.GetActiveDOFIndices())
            return min(abs(vlimits[i] / dqout[i])
                       if dqout[i] != 0. else 1.
                       for i in xrange(vlimits.shape[0])) * dqout

        def CloseEnough():
            """
            The termination condition.
            At each integration step, the geodesic error between the
            start and goal poses is compared. If within threshold,
            the integration will terminate.
            """
            pose_error = util.GetGeodesicDistanceBetweenTransforms(
                manip.GetEndEffectorTransform(), goal_pose)
            if pose_error < pose_error_tol:
                return Status.TERMINATE
            return Status.CONTINUE

        traj = self.FollowVectorField(robot, vf_geodesic, CloseEnough,
                                      integration_interval,
                                      timelimit,
                                      **kw_args)

        # Flag this trajectory as unconstrained. This overwrites the
        # constrained flag set by FollowVectorField.
        util.SetTrajectoryTags(traj, {Tags.CONSTRAINED: False}, append=True)
        return traj

    @ClonedPlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                max_distance=None, timelimit=5.0,
                                position_tolerance=0.01,
                                angular_tolerance=0.15,
                                integration_interval=10.0,
                                **kw_args):
        """
        Plan to a desired end-effector offset with move-hand-straight
        constraint. Movement less than distance will return failure.
        The motion will not move further than max_distance.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param timelimit timeout in seconds
        @param position_tolerance constraint tolerance in meters
        @param angular_tolerance constraint tolerance in radians
        @param integration_interval The time interval to integrate over
        @return traj
        """
        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')
        elif max_distance is not None and max_distance < distance:
            raise ValueError('Max distance is less than minimum distance.')
        elif position_tolerance < 0:
            raise ValueError('Position tolerance must be non-negative.')
        elif angular_tolerance < 0:
            raise ValueError('Angular tolerance must be non-negative.')

        # Normalize the direction vector.
        direction = numpy.array(direction, dtype='float')
        direction = util.NormalizeVector(direction)

        manip = robot.GetActiveManipulator()
        Tstart = manip.GetEndEffectorTransform()

        def vf_straightline():
            """
            Function defining a joint-space vector field.
            """
            twist = util.GeodesicTwist(manip.GetEndEffectorTransform(),
                                       Tstart)
            twist[0:3] = direction

            dqout, _ = util.ComputeJointVelocityFromTwist(
                robot, twist, joint_velocity_limits=numpy.PINF)

            return dqout

        def TerminateMove():
            """
            Function defining the termination condition.

            Fail if deviation larger than position and angular tolerance.
            Succeed if distance moved is larger than max_distance.
            Cache and continue if distance moved is larger than distance.
            """
            from .exceptions import ConstraintViolationPlanningError

            Tnow = manip.GetEndEffectorTransform()

            geodesic_error = util.GeodesicError(Tstart, Tnow)
            orientation_error = geodesic_error[3]
            position_error = geodesic_error[0:3]

            if numpy.fabs(orientation_error) > angular_tolerance:
                raise ConstraintViolationPlanningError(
                    'Deviated from orientation constraint.')
            distance_moved = numpy.dot(position_error, direction)
            position_deviation = numpy.linalg.norm(position_error -
                                                   distance_moved * direction)
            if position_deviation > position_tolerance:
                raise ConstraintViolationPlanningError(
                    'Deviated from straight line constraint.')

            if max_distance is None:
                if distance_moved > distance:
                    return Status.CACHE_AND_TERMINATE
            elif distance_moved > max_distance:
                return Status.TERMINATE
            elif distance_moved >= distance:
                return Status.CACHE_AND_CONTINUE

            return Status.CONTINUE

        return self.FollowVectorField(robot, vf_straightline, TerminateMove,
                                      integration_interval, timelimit,
                                      **kw_args)

    @ClonedPlanningMethod
    def PlanWorkspacePath(self, robot, traj,
                          timelimit=5.0,
                          position_tolerance=0.01,
                          angular_tolerance=0.15,
                          t_step=0.001,
                          Kp_ff=None,
                          Kp_e=None,
                          integration_interval=10.0,
                          **kw_args):
        """
        Plan a configuration space path given a workspace path.
        Trajectory timing information is ignored.

        @param openravepy.Robot      robot: The robot.
        @param openravepy.Trajectory traj:  Workspace trajectory,
                                            represented as an
                                            OpenRAVE AffineTrajectory.
        @param float timelimit: Max planning time (seconds).
        @param float position_tolerance: Constraint tolerance (meters).
        @param float angular_tolerance:  Constraint tolerance (radians).
        @param float t_step: Time step to find vector tanget to current
                             position on the trajectory, using finite
                             differences.
        @param numpy.array Kp_ff: Feed-forward gain.
                                  A 1x6 vector, where first 3 elements
                                  affect the translational velocity,
                                  the last 3 elements affect the
                                  rotational velocity.
        @param numpy.array Kp_e: Error gain.
                                 A 1x6 vector, where first 3 elements
                                 affect the translational velocity,
                                 the last 3 elements affect the
                                 rotational velocity.
        @param integration_interval The time interval to integrate over.

        @return openravepy.Trajectory qtraj: Configuration space path.
        """

        if not util.IsTrajectoryTypeIkParameterizationTransform6D(traj):
            raise ValueError("Trajectory is not a workspace trajectory, it "
                             "must have configuration specification of "
                             "openravepy.IkParameterizationType.Transform6D")

        if util.IsTimedTrajectory(traj):
            raise ValueError("PlanWorkspacePath expected an un-timed "
                             "trajectory.")

        if position_tolerance < 0.0:
            raise ValueError('Position tolerance must be non-negative.')
        elif angular_tolerance < 0.0:
            raise ValueError('Angular tolerance must be non-negative.')

        # Time the trajectory based on its distance
        traj = util.ComputeGeodesicUnitTiming(traj, env=None, alpha=1.0)

        # Set the default gains
        if Kp_ff is None:
            Kp_ff = 0.4 * numpy.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        if Kp_e is None:
            Kp_e = 1.0 * numpy.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        manip = robot.GetActiveManipulator()

        # Get the final end-effector pose
        duration = traj.GetDuration()
        T_ee_goal = openravepy.matrixFromPose(traj.Sample(duration)[0:7])

        def vf_path():
            """
            Function defining a joint-space vector field.
            """
            T_ee_actual = manip.GetEndEffectorTransform()

            # Find where we are on the goal trajectory by finding
            # the the closest point
            (_, t, _) = util.GetMinDistanceBetweenTransformAndWorkspaceTraj(
                T_ee_actual, traj, 0.0005)

            # Get the desired end-effector transform from
            # the goal trajectory
            desired_T_ee = openravepy.matrixFromPose(traj.Sample(t)[0:7])

            # Get the next end-effector transform, using finite-differences
            pose_ee_next = traj.Sample(t + t_step)[0:7]
            desired_T_ee_next = openravepy.matrixFromPose(pose_ee_next)

            # Get the translation tangent to current position
            tangent_vec = desired_T_ee_next[0:3, 3] - desired_T_ee[0:3, 3]
            # Get the translational error
            position_error_vec = desired_T_ee[0:3, 3] - T_ee_actual[0:3, 3]
            # Get the translational error perpendicular to the path
            tangent_trans_error = \
                position_error_vec - numpy.dot(
                    position_error_vec, util.NormalizeVector(tangent_vec))
            tangent_trans_error = numpy.nan_to_num(tangent_trans_error)

            # The twist between the actual end-effector position and
            # where it should be on the goal trajectory
            # (the error term)
            twist_perpendicular = util.GeodesicTwist(T_ee_actual,
                                                     desired_T_ee)
            twist_perpendicular[0:3] = tangent_trans_error

            # The twist tangent to where the end-effector should be
            # on the goal trajectory
            # (the feed-forward term)
            twist_parallel = util.GeodesicTwist(desired_T_ee,
                                                desired_T_ee_next)

            # Normalize the translational and angular velocity of
            # the feed-forward twist
            twist_parallel[0:3] = util.NormalizeVector(twist_parallel[0:3])
            twist_parallel[3:6] = util.NormalizeVector(twist_parallel[3:6])

            # Apply gains
            twist = Kp_e * twist_perpendicular + Kp_ff * twist_parallel

            # Calculate joint velocities using an optimized jacobian
            dqout, _ = util.ComputeJointVelocityFromTwist(
                robot, twist, joint_velocity_limits=numpy.PINF)
            return dqout

        def TerminateMove():
            """
            Function defining the termination condition.

            Fail if deviation larger than position and angular tolerance.
            Succeed if distance moved is larger than max_distance.
            Cache and continue if distance moved is larger than distance.
            """
            from .exceptions import ConstraintViolationPlanningError

            T_ee_curr = manip.GetEndEffectorTransform()

            # Find where we are on the goal trajectory by finding
            # the the closest point
            (_, t, _) = util.GetMinDistanceBetweenTransformAndWorkspaceTraj(
                T_ee_curr, traj, 0.0005)

            # Get the desired end-effector transform from
            # the goal trajectory
            desired_T_ee = openravepy.matrixFromPose(traj.Sample(t)[0:7])

            # Get the position vector tangent to the trajectory,
            # using finite-differences
            pose_ee_next = traj.Sample(t + t_step)[0:7]
            desired_T_ee_next = openravepy.matrixFromPose(pose_ee_next)
            tangent_vec = desired_T_ee_next[0:3, 3] - desired_T_ee[0:3, 3]

            # Calculate error between current end-effector pose
            # and where we should be on the goal trajectory
            geodesic_error = util.GeodesicError(desired_T_ee, T_ee_curr)
            orientation_error = geodesic_error[3]
            position_error_vec = geodesic_error[0:3]

            # Use only the translation error that is perpendicular
            # to our current position
            tangent_trans_error = \
                position_error_vec - numpy.dot(
                    position_error_vec, util.NormalizeVector(tangent_vec))
            tangent_trans_error = numpy.nan_to_num(tangent_trans_error)

            position_error = tangent_trans_error

            if numpy.fabs(orientation_error) > angular_tolerance:
                raise ConstraintViolationPlanningError(
                    'Deviated from orientation constraint.')

            position_deviation = numpy.linalg.norm(position_error)
            if position_deviation > position_tolerance:
                raise ConstraintViolationPlanningError(
                    'Deviated from straight line constraint.')

            # Check if we have reached the end of the goal trajectory
            error_to_goal = util.GeodesicError(T_ee_curr, T_ee_goal)
            orientation_error = error_to_goal[3]  # radians
            position_error = error_to_goal[0:3]  # x,y,z
            if ((numpy.fabs(orientation_error) < angular_tolerance) and
                    (numpy.linalg.norm(position_error) < position_tolerance)):
                return Status.CACHE_AND_TERMINATE

            return Status.CONTINUE

        return self.FollowVectorField(robot, vf_path, TerminateMove,
                                      integration_interval,
                                      timelimit, **kw_args)

    @ClonedPlanningMethod
    def FollowVectorField(self, robot, fn_vectorfield, fn_terminate,
                          integration_time_interval=10.0, timelimit=5.0,
                          sampling_func=util.SampleTimeGenerator,
                          norm_order=2, **kw_args):
        """
        Follow a joint space vectorfield to termination.

        @param robot
        @param fn_vectorfield a vectorfield of joint velocities
        @param fn_terminate custom termination condition
        @param integration_time_interval The time interval to integrate
                                         over.
        @param timelimit time limit before giving up
        @param sampling_func sample generator to compute validity checks
           Note: Function will terminate as soon as invalid configuration is 
                 encountered. No more samples will be requested from the 
                 sampling_func after this occurs.
        @param norm_order order of norm to use for collision checking
        @param kw_args keyword arguments to be passed to fn_vectorfield
        @return traj
        """
        from .exceptions import (
            CollisionPlanningError,
            SelfCollisionPlanningError,
        )
        from openravepy import CollisionReport, RaveCreateTrajectory
        from ..util import GetLinearCollisionCheckPts
        import time
        import scipy.integrate

        CheckLimitsAction = openravepy.KinBody.CheckLimitsAction

        # This is a workaround to emulate 'nonlocal' in Python 2.
        nonlocals = {
            'exception': None,
            't_cache': None,
            't_check': 0.,
        }

        env = robot.GetEnv()
        active_indices = robot.GetActiveDOFIndices()

        # Create a new trajectory matching the current
        # robot's joint configuration specification
        cspec = robot.GetActiveConfigurationSpecification('linear')
        cspec.AddDeltaTimeGroup()
        cspec.ResetGroupOffsets()
        path = RaveCreateTrajectory(env, '')
        path.Init(cspec)

        time_start = time.time()

        def fn_wrapper(t, q):
            """
            The integrator will try to solve this equation
            at each time step.
            Note: t is the integration time and is non-monotonic.
            """
            # Set the joint values, without checking the joint limits
            robot.SetActiveDOFValues(q, CheckLimitsAction.Nothing)

            return fn_vectorfield()

        def fn_status_callback(t, q):
            """
            Check joint-limits and collisions for a specific joint
            configuration. This is called multiple times at DOF
            resolution in order to check along the entire length of the
            trajectory.
            Note: This is called by fn_callback, which is currently
            called after each integration time step, which means we are
            doing more checks than required.
            """
            if time.time() - time_start >= timelimit:
                raise TimeLimitError()

            # Check joint position limits.
            # We do this before setting the joint angles.
            util.CheckJointLimits(robot, q)

            robot.SetActiveDOFValues(q)

            # Check collision (throws an exception on collision)
            robot_checker.VerifyCollisionFree()

            # Check the termination condition.
            status = fn_terminate()

            if Status.DoesCache(status):
                nonlocals['t_cache'] = t

            if Status.DoesTerminate(status):
                raise TerminationError()

        def fn_callback(t, q):
            """
            This is called at every successful integration step.
            """
            try:
                # Add the waypoint to the trajectory.
                waypoint = numpy.zeros(cspec.GetDOF())
                cspec.InsertDeltaTime(waypoint, t - path.GetDuration())
                cspec.InsertJointValues(waypoint, q, robot, active_indices, 0)
                path.Insert(path.GetNumWaypoints(), waypoint)

                # Run constraint checks at DOF resolution.
                if path.GetNumWaypoints() == 1:
                    checks = [(t, q)]
                else:
                    # TODO: This will recheck the entire trajectory
                    #  Ideally should just check the new portion of the trajectory
                    checks = GetLinearCollisionCheckPts(robot, path,
                                                        norm_order=norm_order,
                                                        sampling_func=sampling_func)
                    # start_time=nonlocals['t_check'])

                for t_check, q_check in checks:
                    fn_status_callback(t_check, q_check)

                    # Record the time of this check so we continue checking at
                    # DOF resolution the next time the integrator takes a step.
                    nonlocals['t_check'] = t_check

                return 0  # Keep going.
            except PlanningError as e:
                nonlocals['exception'] = e
                return -1  # Stop.

        with self.robot_collision_checker(robot) as robot_checker:
            # Integrate the vector field to get a configuration space path.
            #
            # TODO: Tune the integrator parameters.
            #
            # Integrator: 'dopri5'
            # DOPRI (Dormand & Prince 1980) is an explicit method for solving ODEs.
            # It is a member of the Runge-Kutta family of solvers.
            integrator = scipy.integrate.ode(f=fn_wrapper)
            integrator.set_integrator(name='dopri5',
                                      first_step=0.1,
                                      atol=1e-3,
                                      rtol=1e-3)
            # Set function to be called at every successful integration step.
            integrator.set_solout(fn_callback)
            integrator.set_initial_value(y=robot.GetActiveDOFValues(), t=0.)

            integrator.integrate(t=integration_time_interval)

        t_cache = nonlocals['t_cache']
        exception = nonlocals['exception']

        if t_cache is None:
            raise exception or PlanningError(
                'An unknown error has occurred.', deterministic=True)
        elif exception:
            logger.warning('Terminated early: %s', str(exception))

        # Remove any parts of the trajectory that are not cached. This also
        # strips the (potentially infeasible) timing information.
        output_cspec = robot.GetActiveConfigurationSpecification('linear')
        output_path = RaveCreateTrajectory(env, '')
        output_path.Init(output_cspec)

        # Add all waypoints before the last integration step. GetWaypoints does
        # not include the upper bound, so this is safe.
        cached_index = path.GetFirstWaypointIndexAfterTime(t_cache)
        output_path.Insert(0, path.GetWaypoints(0, cached_index), cspec)

        # Add a segment for the feasible part of the last integration step.
        output_path.Insert(output_path.GetNumWaypoints(),
                           path.Sample(t_cache),
                           cspec)

        util.SetTrajectoryTags(output_path, {
            Tags.SMOOTH: True,
            Tags.CONSTRAINED: True,
            Tags.DETERMINISTIC_TRAJECTORY: True,
            Tags.DETERMINISTIC_ENDPOINT: True,
        }, append=True)

        return output_path
