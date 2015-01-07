#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#          Pras Velagapudi <pkv@cs.cmu.edu>
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
from .. import ik_ranking
from base import BasePlanner, PlanningError, PlanningMethod

logger = logging.getLogger('prpy.planning.ik')


def unique_rows(a):
    a = numpy.ascontiguousarray(a)
    unique_a = numpy.unique(a.view([('', a.dtype)]*a.shape[1]))
    return unique_a.view(a.dtype).reshape((unique_a.shape[0], a.shape[1]))


class IKPlanner(BasePlanner):
    def __init__(self):
        super(IKPlanner, self).__init__()

    def __str__(self):
        return 'IKPlanner'

    @PlanningMethod
    def PlanToIKs(self, robot, goal_poses,
                  ranker=ik_ranking.JointLimitAvoidance,
                  num_attempts=5, chunk_size=1, **kw_args):
        """
        Create a plan to put a robot active manipulator's end effector at one
        of the specified affine poses.

        To do this, full IK solutions are computed for each goal pose. These
        solutions are combined into a single list, which is ranked by the
        specified ranking function. The solutions are chunked into groups of
        the specified size and the robot planner is called iteratively on each
        group in descending ranked order to find a trajectory to reach any
        solution in the group.

        The planner will terminate when a solution trajectory is found or the
        robot planner has be called for the specified number of attempts.

        If chunk size = 1, robot.planner.PlanToConfiguration is used.
        If chunk size >= 1, robot.planner.PlanToConfigurations is used.

        @param robot the robot whose active manipulator will be used
        @param goal_pose a desired affine transform for the end effector
        @param ranker an IK ranking function to use over the IK solutions
        @param num_attempts the number of planning calls before giving up
        @param chunk_size the number of IK solutions to use per planning call
        @return traj a trajectory that places the end effector at the goal pose
        """
        from openravepy import (IkFilterOptions,
                                IkParameterization,
                                IkParameterizationType)

        # FIXME: Currently meta-planners duplicate IK ranking in each planning
        # thread. It should be possible to fix this by IK ranking once, then
        # calling PlanToConfiguration in separate threads.

        # Find complete list of IK solutions for every specified goal pose.
        with robot.GetEnv():
            ik_solutions = []
            for goal_pose in goal_poses:
                manipulator = robot.GetActiveManipulator()
                ik_param = IkParameterization(
                    goal_pose, IkParameterizationType.Transform6D)
                ik_solution = manipulator.FindIKSolutions(
                    ik_param, IkFilterOptions.CheckEnvCollisions)
                if ik_solution.shape[0] > 0:
                    ik_solutions.append(ik_solution)

        if len(ik_solutions) == 0:
            raise PlanningError('There is no IK solution at the goal pose.')

        # Combine this into a single list of potential goal configurations.
        ik_solutions = numpy.vstack(ik_solutions)
        ik_solutions = unique_rows(ik_solutions)

        # Sort the IK solutions in ascending order by the costs returned by the
        # ranker. Lower cost solutions are better and infinite cost solutions
        # are assumed to be infeasible.
        scores = ranker(robot, ik_solutions)
        ranked_indices = numpy.argsort(scores)
        ranked_indices = ranked_indices[~numpy.isposinf(scores)]
        ranked_ik_solutions = ik_solutions[ranked_indices, :]

        if ranked_ik_solutions.shape[0] == 0:
            raise PlanningError('All IK solutions have infinite cost.')

        # Group the IK solutions into groups of the specified size
        # (plan for each group of IK solutions together).
        ranked_ik_solution_sets = [
            ranked_ik_solutions[i:i+chunk_size, :]
            for i in range(0, ranked_ik_solutions.shape[0], chunk_size)
        ]

        # Sequentially plan to the solution groups in descending cost order.
        num_attempts = min(len(ranked_ik_solution_sets), num_attempts)
        ik_set_list = enumerate(ranked_ik_solution_sets[:num_attempts])
        for i, ik_set in ik_set_list:
            try:
                if ik_set.shape[0] > 1:
                    traj = robot.planner.PlanToConfigurations(robot, ik_set)
                else:
                    traj = robot.planner.PlanToConfiguration(robot, ik_set[0])

                logger.info('Planned to IK solution set %d of %d.',
                            i + 1, num_attempts)
                return traj
            except PlanningError as e:
                logger.warning(
                    'Planning to IK solution set %d of %d failed: %s',
                    i + 1, num_attempts, e)

        raise PlanningError(
            'Planning to the top {:d} of {:d} IK solution sets failed.'
            .format(num_attempts, ranked_ik_solution_sets.shape[0]))

    @PlanningMethod
    def PlanToIK(self, robot, goal_pose, **kw_args):
        """
        Create a plan to put a robot active manipulator's end effector at the
        specified affine pose.

        @param robot the robot whose active manipulator will be used
        @param goal_pose a desired affine transform for the end effector
        @param ranker an IK ranking function to use over the IK solutions
        @param num_attempts the number of planning calls before giving up
        @param chunk_size the number of IK solutions to use per planning call
        @return traj a trajectory that places the end effector at the goal pose
        """
        return self.PlanToIKs(robot, goal_poses=[goal_pose], **kw_args)
