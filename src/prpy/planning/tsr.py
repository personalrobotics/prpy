#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Pras Velagapudi <pkv@cs.cmu.edu>
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
import time
import itertools
import numpy
from base import BasePlanner, PlanningMethod, PlanningError

logger = logging.getLogger('prpy.planning.tsr')


class TSRPlanner(BasePlanner):
    def __init__(self):
        super(TSRPlanner, self).__init__()

    def __str__(self):
        return 'TSRPlanner'

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, num_attempts=10,
                  chunk_size=20, ranker=None,
                  tsr_samples=100, tsr_timeout=1.0, **kw_args):
        """
        Plan to a desired TSR set using a-priori goal sampling.  This planner
        samples a fixed number of goals from the specified TSRs up-front, then
        uses robot.planner.PlanToIK to attempt to plan to the resulting affine
        transformations.

        This planner will return failure if the provided TSR chains require
        any constraint other than goal sampling.

        @param robot the robot whose active manipulator will be used
        @param tsrchains a list of TSR chains that define a goal set
        @param num_attempts the maximum number of planning attempts to make
        @param chunk_size the number of sampled goals to use per planning call
        @param tsr_samples the maximum number of samples of the goal TSR chains
        @param tsr_timeout the maximum time to spend sampling goal TSR chains
        @param ranker an IK ranking function to use over the IK solutions
        @return traj a trajectory that satisfies the specified TSR chains
        """
        from ..ik_ranking import NominalConfiguration
        if ranker is None:
            ranker = NominalConfiguration(robot.GetActiveDOFValues())

        # Plan using the active manipulator.
        manipulator = robot.GetActiveManipulator()

        # Create an iterator that will cycle through sampling each TSR chain.
        tsr_cycler = itertools.cycle(tsrchains)

        # Create an iterator that will sample until the timelimit.
        tsr_timelimit = time.time() + tsr_timeout
        tsr_sampler = itertools.takewhile(
            lambda v: time.time() < tsr_timelimit, tsr_cycler)

        # Sample a list of TSR poses and collate valid IK solutions.
        from openravepy import (IkFilterOptions,
                                IkParameterization,
                                IkParameterizationType)
        ik_solutions = []
        for tsrchain in tsr_sampler:
            ik_param = IkParameterization(
                tsrchain.sample(), IkParameterizationType.Transform6D)
            ik_solution = manipulator.FindIKSolutions(
                ik_param, IkFilterOptions.CheckEnvCollisions)
            if ik_solution.shape[0] > 0:
                ik_solutions.append(ik_solution)

        if len(ik_solutions) == 0:
            raise PlanningError('No collision-free IK solutions at goal TSRs.')

        # Sort the IK solutions in ascending order by the costs returned by the
        # ranker. Lower cost solutions are better and infinite cost solutions
        # are assumed to be infeasible.
        ik_solutions = numpy.vstack(ik_solutions)
        scores = ranker(robot, ik_solutions)
        ranked_indices = numpy.argsort(scores)
        ranked_indices = ranked_indices[~numpy.isposinf(scores)]
        ranked_ik_solutions = ik_solutions[ranked_indices, :]

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

        # If none of the planning attempts succeeded, report failure.
        raise PlanningError(
            'Planning to the top {:d} of {:d} IK solution sets failed.'
            .format(num_attempts, len(ranked_ik_solution_sets)))
