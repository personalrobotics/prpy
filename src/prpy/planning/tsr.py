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
from base import BasePlanner, PlanningMethod

logger = logging.getLogger('prpy.planning.tsr')


class TSRPlanner(BasePlanner):
    def __init__(self):
        super(TSRPlanner, self).__init__()

    def __str__(self):
        return 'TSRPlanner'

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, chunk_size=10,
                  tsr_samples=10, ranker=None, **kw_args):
        """
        Plan to a desired TSR set using a-priori goal sampling.  This planner
        samples a fixed number of goals from the specified TSRs up-front, then
        uses robot.planner.PlanToIK to attempt to plan to the resulting affine
        transformations.

        This planner will return failure if the provided TSR chains require
        any constraint other than goal sampling.

        @param robot the robot whose active manipulator will be used
        @param tsrchains a list of TSR chains that define a goal set
        @param tsr_samples the number of samples of each goal TSR chain to try
        @param chunk_size the number of possible goals to use per planning call
        @param ranker an IK ranking function to use over the IK solutions
        @return traj a trajectory that satisfies the specified TSR chains
        """
        from ..ik_ranking import NominalConfiguration
        if ranker is None:
            ranker = NominalConfiguration(robot.GetActiveDOFValues())

        # TODO: check that the TSR chains only require sample_goal.
        # TODO: round robin between each TSR chain that has sample_goal.
        goal_poses = [tsrchains[0].sample() for i in range(tsr_samples)]
        return robot.planner.PlanToIKs(
            robot, goal_poses,
            ranker=ranker, chunk_size=chunk_size, **kw_args)
