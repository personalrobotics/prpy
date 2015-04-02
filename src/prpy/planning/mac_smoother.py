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

from ..util import CopyTrajectory, SimplifyTrajectory, SetTrajectoryTags
from base import (BasePlanner, PlanningError, PlanningMethod,
                  UnsupportedPlanningError, Tags)
from openravepy import (Planner, PlannerStatus, RaveCreatePlanner,
                        openrave_exception)


class MacSmoother(BasePlanner):
    def __init__(self):
        super(MacSmoother, self).__init__()

        self.blender = RaveCreatePlanner(self.env, 'PrSplineMacBlender')
        if self.blender is None:
            raise UnsupportedPlanningError(
                'Unable to create PrSplineMacBlender planner. Is or_pr_spline'
                ' installed and in your OPENRAVE_PLUGIN path?'
            )
            
        self.retimer = RaveCreatePlanner(self.env, 'PrSplineMacTimer')
        if self.retimer is None:
            raise UnsupportedPlanningError(
                'Unable to create PrSplineMacRetimer planner. Is or_pr_spline'
                ' installed and in your OPENRAVE_PLUGIN path?'
            )

    @PlanningMethod
    def RetimeTrajectory(self, robot, path):
        # Copy the input trajectory into the planning environment. This is
        # necessary for two reasons: (1) the input trajectory may be in another
        # environment and/or (2) the retimer modifies the trajectory in-place.
        output_traj = CopyTrajectory(path, env=self.env)
        output_traj = SimplifyTrajectory(output_traj, robot)

        # Blend the piecewise-linear input trajectory. The blender outputs a
        # collision-free path, consisting of piecewise-linear segments and
        # quintic blends through waypoints.
        try:
            params = Planner.PlannerParameters()
            self.blender.InitPlan(robot, params)
            status = self.blender.PlanPath(output_traj)
            if status not in [ PlannerStatus.HasSolution,
                               PlannerStatus.InterruptedWithSolution ]:
                raise PlanningError('Blending trajectory failed.')
        except openrave_exception as e:
            raise PlanningError('Blending trajectory failed: ' + str(e))

        # Find the time-optimal trajectory that follows the blended path
        # subject to joint velocity and acceleration limits.
        try:
            params = Planner.PlannerParameters()
            self.retimer.InitPlan(robot, params)
            status = self.retimer.PlanPath(output_traj)
            if status not in [ PlannerStatus.HasSolution,
                               PlannerStatus.InterruptedWithSolution ]:
                raise PlanningError('Timing trajectory failed.')
        except openrave_exception as e:
            raise PlanningError('Timing trajectory failed: ' + str(e))

        SetTrajectoryTags(output_traj, {Tags.SMOOTH: True}, append=True)
        return output_traj

