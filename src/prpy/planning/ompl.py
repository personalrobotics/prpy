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
from ..util import CopyTrajectory, SetTrajectoryTags
from base import (BasePlanner, PlanningError, UnsupportedPlanningError,
                  PlanningMethod, Tags)
from openravepy import PlannerStatus
from .cbirrt import SerializeTSRChain

logger = logging.getLogger(__name__)


class OMPLPlanner(BasePlanner):
    def __init__(self, algorithm='RRTConnect'):
        super(OMPLPlanner, self).__init__()

        self.setup = False
        self.algorithm = algorithm

        planner_name = 'OMPL_{:s}'.format(algorithm)
        self.planner = openravepy.RaveCreatePlanner(self.env, planner_name)

        if self.planner is None:
            raise UnsupportedPlanningError(
                'Unable to create "{:s}" planner. Is or_ompl installed?'
                .format(planner_name))

    def __str__(self):
        return 'OMPL {0:s}'.format(self.algorithm)

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a desired configuration with OMPL. This will invoke the OMPL
        planner specified in the OMPLPlanner constructor.
        @param robot
        @param goal desired configuration
        @return traj
        """
        return self._Plan(robot, goal=goal, **kw_args)

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        """
        Plan using the given set of TSR chains with OMPL.
        @param robot
        @param tsrchains A list of tsrchains to use during planning
        @param return traj
        """
        return self._TSRPlan(robot, tsrchains, **kw_args)

    def _Plan(self, robot, goal=None, timeout=30., shortcut_timeout=5.,
              continue_planner=False, ompl_args=None,
              formatted_extra_params=None, **kw_args):
        extraParams = '<time_limit>{:f}</time_limit>'.format(timeout)

        if ompl_args is not None:
            for key, value in ompl_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(
                    k=str(key), v=str(value))

        if formatted_extra_params is not None:
            extraParams += formatted_extra_params

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        if goal is not None:
            params.SetGoalConfig(goal)
        params.SetExtraParameters(extraParams)

        traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')

        # Plan.
        with self.env:
            if (not continue_planner) or (not self.setup):
                self.planner.InitPlan(robot, params)
                self.setup = True

            status = self.planner.PlanPath(traj, releasegil=True)
            if status not in [PlannerStatus.HasSolution,
                              PlannerStatus.InterruptedWithSolution]:
                raise PlanningError('Planner returned with status {0:s}.'
                                    .format(str(status)))

        return traj

    def _TSRPlan(self, robot, tsrchains, **kw_args):
        extraParams = ''
        for chain in tsrchains:
            extraParams += '<{k:s}>{v:s}</{k:s}>'.format(
                k='tsr_chain', v=SerializeTSRChain(chain))

        return self._Plan(robot, formatted_extra_params=extraParams, **kw_args)


class RRTConnect(OMPLPlanner):
    def __init__(self):
        OMPLPlanner.__init__(self, algorithm='RRTConnect')

    def _SetPlannerRange(self, robot, ompl_args=None):
        from copy import deepcopy

        if ompl_args is None:
            ompl_args = dict()
        else:
            ompl_args = deepcopy(ompl_args)

        # Set the default range to the collision checking resolution.
        if 'range' not in ompl_args:
            epsilon = 1e-6

            # Compute the collision checking resolution as a conservative
            # fraction of the state space extents. This mimics the logic inside
            # or_ompl used to set collision checking resolution.
            dof_limit_lower, dof_limit_upper = robot.GetActiveDOFLimits()
            dof_ranges = dof_limit_upper - dof_limit_lower
            dof_resolution = robot.GetActiveDOFResolutions()
            ratios = dof_resolution / dof_ranges
            conservative_ratio = numpy.min(ratios)

            # Convert the ratio back to a collision checking resolution. Set
            # RRT-Connect's range to the same value used for collision
            # detection inside OpenRAVE.
            longest_extent = numpy.max(dof_ranges)
            ompl_range = conservative_ratio * longest_extent - epsilon

            ompl_args['range'] = ompl_range
            logger.debug('Defaulted RRT-Connect range parameter to %.3f.',
                         ompl_range)
        return ompl_args

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, ompl_args=None, **kw_args):
        """
        Plan to a desired configuration with OMPL. This will invoke the OMPL
        planner specified in the OMPLPlanner constructor.
        @param robot
        @param goal desired configuration
        @return traj
        """
        ompl_args = self._SetPlannerRange(robot, ompl_args=ompl_args)
        return self._Plan(robot, goal=goal, ompl_args=ompl_args, **kw_args)

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, ompl_args=None, **kw_args):
        """
        Plan using the given TSR chains with OMPL.
        @param robot
        @param tsrchains A list of TSRChain objects to respect during planning
        @param ompl_args ompl RRTConnect specific parameters
        @return traj
        """
        ompl_args = self._SetPlannerRange(robot, ompl_args=ompl_args)
        return self._TSRPlan(robot, tsrchains, ompl_args=ompl_args, **kw_args)


class OMPLSimplifier(BasePlanner):
    def __init__(self):
        super(OMPLSimplifier, self).__init__()

        planner_name = 'OMPL_Simplifier'
        self.planner = openravepy.RaveCreatePlanner(self.env, planner_name)

        if self.planner is None:
            raise UnsupportedPlanningError(
                'Unable to create OMPL_Simplifier planner.')

    def __str__(self):
        return 'OMPL Simplifier'

    @PlanningMethod
    def ShortcutPath(self, robot, path, timeout=1., **kwargs):
        # The planner operates in-place, so we need to copy the input path. We
        # also need to copy the trajectory into the planning environment.
        output_path = CopyTrajectory(path, env=self.env)

        extraParams = '<time_limit>{:f}</time_limit>'.format(timeout)

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetExtraParameters(extraParams)

        # TODO: It would be nice to call planningutils.SmoothTrajectory here,
        # but we can't because it passes a NULL robot to InitPlan. This is an
        # issue that needs to be fixed in or_ompl.
        self.planner.InitPlan(robot, params)
        status = self.planner.PlanPath(output_path, releasegil=True)
        if status not in [PlannerStatus.HasSolution,
                          PlannerStatus.InterruptedWithSolution]:
            raise PlanningError('Simplifier returned with status {0:s}.'
                                .format(str(status)))

        SetTrajectoryTags(output_path, {Tags.SMOOTH: True}, append=True)
        return output_path
