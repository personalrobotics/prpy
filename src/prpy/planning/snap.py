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
import numpy
import openravepy
from ..util import SetTrajectoryTags
from base import BasePlanner, PlanningError, PlanningMethod, Tags


class SnapPlanner(BasePlanner):
    """Planner that checks the straight-line trajectory to the goal.

    SnapPlanner is a utility planner class that collision checks the
    straight-line trajectory to the goal. If that trajectory is invalid, i.e..
    due to an environment or self collision, the planner immediately returns
    failure by raising a PlanningError. Collision checking is performed using
    the standard CheckPathAllConstraints method.

    SnapPlanner is intended to be used only as a "short circuit" to speed up
    planning between nearby configurations. This planner is most commonly used
    as the first item in a Sequence meta-planner to avoid calling a motion
    planner when the trivial solution is valid.
    """
    def __init__(self):
        super(SnapPlanner, self).__init__()

    def __str__(self):
        return 'SnapPlanner'

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Attempt to plan a straight line trajectory from the robot's current
        configuration to the goal configuration. This will fail if the
        straight-line path is not collision free.

        @param robot
        @param goal desired configuration
        @return traj
        """
        return self._Snap(robot, goal, **kw_args)

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, **kw_args):
        """
        Attempt to plan a straight line trajectory from the robot's current
        configuration to a desired end-effector pose. This happens by finding
        the closest IK solution to the robot's current configuration and
        attempts to snap there (using PlanToConfiguration) if possible. In the
        case of a redundant manipulator, no attempt is made to check other IK
        solutions.

        @param robot
        @param goal_pose desired end-effector pose
        @return traj
        """
        
        from .exceptions import CollisionPlanningError,SelfCollisionPlanningError
        from openravepy import CollisionReport
        
        ikp = openravepy.IkParameterizationType
        ikfo = openravepy.IkFilterOptions

        # Find an IK solution. OpenRAVE tries to return a solution that is
        # close to the configuration of the arm, so we don't need to do any
        # custom IK ranking.
        manipulator = robot.GetActiveManipulator()
        ik_param = openravepy.IkParameterization(goal_pose, ikp.Transform6D)
        ik_solution = manipulator.FindIKSolution(
            ik_param, ikfo.CheckEnvCollisions,
            ikreturn=False, releasegil=True
        )
        
        if ik_solution is None: 
            # FindIKSolutions is slower than FindIKSolution, 
            # so call this only to identify and raise error when there is no solution
            ik_solutions = manipulator.FindIKSolutions(
                    ik_param,
                    ikfo.IgnoreSelfCollisions,
                    ikreturn=False,
                    releasegil=True
                )

            for q in ik_solutions:
                robot.SetActiveDOFValues(q)
                report = CollisionReport()
                if self.env.CheckCollision(robot, report=report):
                    raise CollisionPlanningError.FromReport(report) 
                elif robot.CheckSelfCollision(report=report):
                    raise SelfCollisionPlanningError.FromReport(report)
            
            raise PlanningError('There is no IK solution at the goal pose.')

        return self._Snap(robot, ik_solution, **kw_args)

    def _Snap(self, robot, goal, **kw_args):
        from .exceptions import CollisionPlanningError,SelfCollisionPlanningError,JointLimitError
        from openravepy import CollisionReport

        Closed = openravepy.Interval.Closed

        start = robot.GetActiveDOFValues()
        active_indices = robot.GetActiveDOFIndices()

        # Use the CheckPathAllConstraints helper function to collision check
        # the straight-line trajectory. We pass dummy values for dq0, dq1,
        # and timeelapsed since this is a purely geometric check.
        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(goal)
        check = params.CheckPathAllConstraints(start, goal, [], [], 0., Closed,
            options = 15, returnconfigurations = True)

        # If valid, above function returns (0,None, None, 0) 
        # if not, it returns (1, invalid_config ,[], time_when_invalid),
        # in which case we identify and raise appropriate error. 
        if check[0] != 0:
            q = check[1]

            # Check for joint limit violation
            q_limit_min, q_limit_max = robot.GetActiveDOFLimits()

            lower_position_violations = (q < q_limit_min)
            if lower_position_violations.any():
                index = lower_position_violations.nonzero()[0][0]
                raise JointLimitError(robot,
                    dof_index=active_indices[index],
                    dof_value=q[index],
                    dof_limit=q_limit_min[index],
                    description='position')

            upper_position_violations = (q> q_limit_max)
            if upper_position_violations.any():
                index = upper_position_violations.nonzero()[0][0]
                raise JointLimitError(robot,
                    dof_index=active_indices[index],
                    dof_value=q[index],
                    dof_limit=q_limit_max[index],
                    description='position')

            # Check for collision
            robot.SetActiveDOFValues(q)
            report = CollisionReport()
            if self.env.CheckCollision(robot, report=report):
                raise CollisionPlanningError.FromReport(report) 
            elif robot.CheckSelfCollision(report=report):
                raise SelfCollisionPlanningError.FromReport(report)

            raise PlanningError('Straight line trajectory is not valid.' )

        # Create a two-point trajectory that starts at our current
        # configuration and takes us to the goal.
        traj = openravepy.RaveCreateTrajectory(self.env, '')
        cspec = robot.GetActiveConfigurationSpecification('linear')
        active_indices = robot.GetActiveDOFIndices()

        start_waypoint = numpy.zeros(cspec.GetDOF())
        cspec.InsertJointValues(start_waypoint, start, robot,
                                active_indices, False)

        traj.Init(cspec)
        traj.Insert(0, start_waypoint.ravel())

        # Make the trajectory end at the goal configuration, as long as it
        # was not identical to the start configuration.
        if not numpy.allclose(start, goal):
            goal_waypoint = numpy.zeros(cspec.GetDOF())
            cspec.InsertJointValues(goal_waypoint, goal, robot,
                                    active_indices, False)
            traj.Insert(1, goal_waypoint.ravel())

        # Tag the return trajectory as smooth (in joint space) and return it.
        SetTrajectoryTags(traj, {Tags.SMOOTH: True}, append=True)
        return traj
