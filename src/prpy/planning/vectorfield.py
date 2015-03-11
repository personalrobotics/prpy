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

        def vf_geodesic(robot, goal_pose):
            manip = robot.GetActiveManipulator()
            return prpy.util.GeodesicTwist(manip.GetEndEffectorTransform(),
                                           goal_pose)

        qtraj = self.FollowVectorField(robot, vf_geodesic, timelimit,
                                       goal_pose=goal_pose)

        # Check if the last waypoint is at the goal
        with robot:
            cspec = qtraj.GetConfigurationSpecification()
            waypoint_end = qtraj.GetWaypoint(qtraj.GetNumWaypoints()-1)
            qend = cspec.ExtractJointValues(waypoint_end,
                                            robot,
                                            range(robot.GetDOF()),
                                            0)
            robot.SetActiveDOFValues(qend)
            manip = robot.GetActiveManipulator()
            pose_error = prpy.util.GeodesicDistance(
                                    manip.GetEndEffectorTransform(),
                                    goal_pose)
            if pose_error > pose_error_tol:
                raise PlanningError('Local minimum: \
                    unable to reach goal pose: ')
        return qtraj

    @PlanningMethod
    def FollowVectorField(self, robot, fn_vectorfield, timelimit=5.0,
                          dq_tol=0.0001, **kw_args):
        """
        Plan to an end effector pose by following a geodesic loss function
        in SE(3) via an optimized Jacobian.

        @param robot
        @param fn_vectorfield inputs robot, kw_args and outputs velocity twist
        @param timelimit time limit before giving up
        @param dq_tol velocity tolerance for termination
        @param kw_args keyword arguments to be passed to fn_vectorfield
        @return traj
        """
        start_time = time.time()

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
                twist = fn_vectorfield(robot, **kw_args)
                dqout, tout = prpy.util.ComputeJointVelocityFromTwist(
                                robot, twist)
                if (numpy.linalg.norm(dqout) < dq_tol):
                    break
                qnew = q_curr + dqout*dt
                robot.SetActiveDOFValues(qnew)

        return qtraj
