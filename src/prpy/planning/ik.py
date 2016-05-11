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
import logging
import numpy
import openravepy
from .. import ik_ranking
from base import (BasePlanner,
                  PlanningError,
                  ClonedPlanningMethod)

logger = logging.getLogger(__name__)


class IKPlanner(BasePlanner):
    def __init__(self, delegate_planner=None):
        super(IKPlanner, self).__init__()
        self.delegate_planner = delegate_planner

    def __str__(self):
        return 'IKPlanner'

    @ClonedPlanningMethod
    def PlanToIK(self, robot, goal_pose, ranker=ik_ranking.JointLimitAvoidance,
                 num_attempts=1, **kw_args):
        from openravepy import (IkFilterOptions,
                                IkParameterization,
                                IkParameterizationType)

        # FIXME: Currently meta-planners duplicate IK ranking in each planning
        # thread. It should be possible to fix this by IK ranking once, then
        # calling PlanToConfiguration in separate threads.

        # Find an unordered list of IK solutions.
        with robot.GetEnv():
            manipulator = robot.GetActiveManipulator()
            ik_param = IkParameterization(
                goal_pose, IkParameterizationType.Transform6D)
            ik_solutions = manipulator.FindIKSolutions(
                ik_param, IkFilterOptions.CheckEnvCollisions,
                ikreturn=False, releasegil=True
            )

        if ik_solutions.shape[0] == 0:
            raise PlanningError('There is no IK solution at the goal pose.')

        # Sort the IK solutions in ascending order by the costs returned by the
        # ranker. Lower cost solutions are better and infinite cost solutions
        # are assumed to be infeasible.
        scores = ranker(robot, ik_solutions)
        ranked_indices = numpy.argsort(scores)
        ranked_indices = ranked_indices[~numpy.isposinf(scores)]
        ranked_ik_solutions = ik_solutions[ranked_indices, :]

        if ranked_ik_solutions.shape[0] == 0:
            raise PlanningError('All IK solutions have infinite cost.')

        # Sequentially plan to the solutions in descending order of cost.
        planner = self.delegate_planner or robot.planner
        p = openravepy.KinBody.SaveParameters

        with robot.CreateRobotStateSaver(p.ActiveDOF):
            robot.SetActiveDOFs(manipulator.GetArmIndices())

            num_attempts = min(ranked_ik_solutions.shape[0], num_attempts)
            for i, ik_sol in enumerate(ranked_ik_solutions[0:num_attempts, :]):
                try:
                    traj = planner.PlanToConfiguration(robot, ik_sol)
                    logger.info('Planned to IK solution %d of %d.',
                                i + 1, num_attempts)
                    return traj
                except PlanningError as e:
                    logger.warning(
                        'Planning to IK solution %d of %d failed: %s',
                        i + 1, num_attempts, e)

        raise PlanningError(
            'Planning to the top {:d} of {:d} IK solutions failed.'
            .format(num_attempts, ranked_ik_solutions.shape[0]))
