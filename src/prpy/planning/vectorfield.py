#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: Siddhartha Srinivasa <siddh@cs.cmu.edu>
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
import time
from base import BasePlanner, PlanningError, PlanningMethod
import prpy.util

logger = logging.getLogger('planning')


class VectorFieldPlanner(BasePlanner):
    def __init__(self):
        super(VectorFieldPlanner, self).__init__()

    def __str__(self):
        return 'VectorFieldPlanner'

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, timelimit=5.0,
                              pose_error_tol=0.01, **kw_args):
        """
        Plan to an end effector pose by following a geodesic loss function
        in SE(3) via an optimized Jacobian.

        @param robot
        @param goal_pose desired end-effector pose
        @param timelimit time limit before giving up
        @param pose_error_tol in meters
        @return traj
        """
        manip = robot.GetActiveManipulator()

        def vf_geodesic():
            twist = prpy.util.GeodesicTwist(manip.GetEndEffectorTransform(),
                                            goal_pose)
            dqout, tout = prpy.util.ComputeJointVelocityFromTwist(
                                robot, twist)
            return dqout

        def CloseEnough():
            pose_error = prpy.util.GeodesicDistance(
                        manip.GetEndEffectorTransform(),
                        goal_pose)
            if pose_error < pose_error_tol:
                return True
            return False

        qtraj, terminate = self.FollowVectorField(robot, vf_geodesic,
                                                  timelimit, CloseEnough)

        if not terminate:
            raise PlanningError('Local minimum: unable to reach goal pose: ')

        return qtraj

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                max_distance=None, timelimit=5.0,
                                position_tolerance=0.01,
                                angular_tolerance=0.15,
                                **kw_args):
        """
        Plan to a desired end-effector offset with move-hand-straight
        constraint. movement less than distance will return failure. The motion
        will not move further than max_distance.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param timelimit timeout in seconds
        @param position_tolerance constraint tolerance in meters
        @param angular_tolerance constraint tolerance in radians
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
        direction /= numpy.linalg.norm(direction)

        # Default to moving an exact distance.
        if max_distance is None:
            max_distance = distance

        manip = robot.GetActiveManipulator()
        Tstart = manip.GetEndEffectorTransform()

        def vf_straightline():
            twist = prpy.util.GeodesicTwist(manip.GetEndEffectorTransform(),
                                            Tstart)
            twist[0:3] = direction
            dqout, tout = prpy.util.ComputeJointVelocityFromTwist(
                    robot, twist)
            return dqout

        def TerminateMove():
            '''
            Fail if deviation larger than position and angular tolerance.
            Succeed if distance moved is larger than max_distance
            '''
            Tnow = manip.GetEndEffectorTransform()
            error = prpy.util.GeodesicError(Tstart, Tnow)
            if error[3] > angular_tolerance:
                raise PlanningError('Deviated from orientation constraint.')
            distance_moved = numpy.dot(error[0:3], direction)
            position_deviation = numpy.linalg.norm(error[0:3] -
                                                   distance_moved*direction)
            if position_deviation > position_tolerance:
                raise PlanningError('Deviated from straight line constraint.')

            if distance_moved > max_distance:
                return True
            return False

        qtraj, terminate = self.FollowVectorField(robot, vf_straightline,
                                                  timelimit, TerminateMove)

        if not terminate:
            raise PlanningError('Local minimum: unable to reach goal pose: ')

        return qtraj

    @PlanningMethod
    def FollowVectorField(self, robot, fn_vectorfield, timelimit=5.0,
                          fn_terminate=None, dq_tol=0.0001, **kw_args):
        """
        Follow a joint space vectorfield to local minimum or termination.

        @param robot
        @param fn_vectorfield a vectorfield of joint velocities
        @param fn_terminate custom termination condition
        @param timelimit time limit before giving up
        @param dq_tol velocity tolerance for termination
        @param kw_args keyword arguments to be passed to fn_vectorfield
        @return traj
        @return terminate a Boolean that returns the final fn_terminate
        """
        start_time = time.time()

        if fn_terminate is None:
            def fn_terminate():
                return False

        with robot:
            manip = robot.GetActiveManipulator()
            robot.SetActiveDOFs(manip.GetArmIndices())
            # Populate joint positions and joint velocities
            cspec = manip.GetArmConfigurationSpecification('quadratic')
            cspec.AddDerivativeGroups(1, False)
            cspec.AddDeltaTimeGroup()
            cspec.ResetGroupOffsets()
            qtraj = openravepy.RaveCreateTrajectory(self.env,
                                                    'GenericTrajectory')
            qtraj.Init(cspec)

            dqout = robot.GetActiveDOFVelocities()
            dt = min(robot.GetDOFResolutions()/robot.GetDOFVelocityLimits())
            while True:
                # Check for a timeout.
                current_time = time.time()
                if (timelimit is not None and
                        current_time - start_time > timelimit):
                    raise PlanningError('Reached time limit.')

                # Check for collisions.
                if self.env.CheckCollision(robot):
                    raise PlanningError('Encountered collision.')
                if robot.CheckSelfCollision():
                    raise PlanningError('Encountered self-collision.')

                # Add to trajectory
                waypoint = []
                q_curr = robot.GetActiveDOFValues()
                waypoint.append(q_curr)  # joint position
                waypoint.append(dqout)   # joint velocity
                waypoint.append([dt])    # delta time
                waypoint = numpy.concatenate(waypoint)
                qtraj.Insert(qtraj.GetNumWaypoints(), waypoint)
                dqout = fn_vectorfield()
                terminate = fn_terminate()
                if (numpy.linalg.norm(dqout) < dq_tol) or terminate:
                    break
                qnew = q_curr + dqout*dt
                robot.SetActiveDOFValues(qnew)

        return qtraj, terminate
