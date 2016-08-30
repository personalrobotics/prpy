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
import warnings
from ..util import CopyTrajectory, SetTrajectoryTags
from copy import deepcopy
from ..util import CopyTrajectory, SetTrajectoryTags, GetManipulatorIndex
from ..tsr.tsr import TSR, TSRChain
from base import (BasePlanner, PlanningError, UnsupportedPlanningError,
                  ClonedPlanningMethod, Tags)
from openravepy import (
    CollisionOptionsStateSaver,
    PlannerStatus,
)
from ..collision import (
    BakedRobotCollisionCheckerFactory,
    DefaultRobotCollisionCheckerFactory,
    SimpleRobotCollisionCheckerFactory,
)
from .cbirrt import SerializeTSRChain

logger = logging.getLogger(__name__)


class OMPLPlanner(BasePlanner):
    def __init__(self, algorithm='RRTConnect', robot_checker_factory=None):
        super(OMPLPlanner, self).__init__()

        if robot_checker_factory is None:
            robot_checker_factory = DefaultRobotCollisionCheckerFactory

        self.setup = False
        self.algorithm = algorithm

        self.robot_checker_factory = robot_checker_factory

        if isinstance(robot_checker_factory, SimpleRobotCollisionCheckerFactory):
            self._is_baked = False
        elif isinstance(robot_checker_factory, BakedRobotCollisionCheckerFactory):
            self._is_baked = True
        else:
            raise NotImplementedError(
                'or_ompl only supports Simple and'
                ' BakedRobotCollisionCheckerFactory.')

        planner_name = 'OMPL_{:s}'.format(algorithm)
        self.planner = openravepy.RaveCreatePlanner(self.env, planner_name)

        if self.planner is None:
            raise UnsupportedPlanningError(
                'Unable to create "{:s}" planner. Is or_ompl installed?'
                .format(planner_name))

    def __str__(self):
        return 'OMPL {0:s}'.format(self.algorithm)

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a desired configuration with OMPL. This will invoke the OMPL
        planner specified in the OMPLPlanner constructor.
        @param robot
        @param goal desired configuration
        @return traj
        """
        return self._Plan(robot, goal=goal, **kw_args)

    @ClonedPlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        """
        Plan using the given set of TSR chains with OMPL.
        @param robot
        @param tsrchains A list of tsrchains to use during planning
        @param return traj
        """
        return self._Plan(robot, tsrchains=tsrchains, **kw_args)

    def _Plan(self, robot, goal=None, tsrchains=None, timeout=30., shortcut_timeout=5.,
              continue_planner=False, ompl_args=None,
              formatted_extra_params=None, **kw_args):
        env = robot.GetEnv()
        extraParams = '<time_limit>{:f}</time_limit>'.format(timeout)

        if ompl_args is not None:
            for key, value in ompl_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(
                    k=str(key), v=str(value))

        if tsrchains is not None:
            for chain in tsrchains:
                if chain.constrain or chain.sample_start: 
                    raise UnsupportedPlanningError('Only goal tsr is supported by OMPL.')
                    
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(
                    k='tsr_chain', v=SerializeTSRChain(chain))

        if formatted_extra_params is not None:
            extraParams += formatted_extra_params

        if self._is_baked and (ompl_args is None or 'do_baked' not in ompl_args):
            extraParams += '<do_baked>1</do_baked>'

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        if goal is not None:
            params.SetGoalConfig(goal)
        params.SetExtraParameters(extraParams)

        traj = openravepy.RaveCreateTrajectory(env, 'GenericTrajectory')

        # Plan.
        with env:
            if (not continue_planner) or (not self.setup):
                self.planner.InitPlan(robot, params)
                self.setup = True

            # Bypass the context manager since or_ompl does its own baking.
            robot_checker = self.robot_checker_factory(robot)
            options = robot_checker.collision_options
            with CollisionOptionsStateSaver(env.GetCollisionChecker(), options):
                status = self.planner.PlanPath(traj, releasegil=True)

            if status not in [PlannerStatus.HasSolution,
                              PlannerStatus.InterruptedWithSolution]:
                raise PlanningError(
                    'Planner returned with status {:s}.'.format(str(status)),
                    deterministic=False)

        # Tag the trajectory as non-determistic since most OMPL planners are
        # randomized. Additionally tag the goal as non-deterministic if OMPL
        # chose from a set of more than one goal configuration.
        SetTrajectoryTags(traj, {
            Tags.DETERMINISTIC_TRAJECTORY: False,
            Tags.DETERMINISTIC_ENDPOINT: tsrchains is None,
        }, append=True)

        return traj


class OMPLRangedPlanner(OMPLPlanner):
    """ Construct an OMPL planner with a default 'range' parameter set.

    This class wraps OMPLPlanner by setting the default 'range' parameter,
    which defines the length of an extension in algorithms like RRT-Connect, to
    include an fixed number of state validity checks.
    
    This number may be specified explicitly (using 'multiplier') or as an
    approximate fraction of the longest extent of the state space (using
    'fraction'). Exactly one of 'multiplier' or 'fraction' is required.

    @param algorithm name of the OMPL planner to create
    @param robot_checker_factory robot collision checker factory
    @param multiplier number of state validity checks per extension
    @param fraction approximate fraction of the maximum extent of the state
                    space per extension
    """
    def __init__(self, multiplier=None, fraction=None,
                 algorithm='RRTConnect', robot_checker_factory=None):
        if multiplier is None and fraction is None:
            raise ValueError('Either "multiplier" of "fraction" is required.')
        if multiplier is not None and fraction is not None:
            raise ValueError('"multiplier" and "fraction" are exclusive.')
        if multiplier is not None and not (multiplier >= 1):
            raise ValueError('"mutiplier" must be a positive integer.')
        if fraction is not None and not (0. <= fraction <= 1.):
            raise ValueError('"fraction" must be between zero and one.')

        OMPLPlanner.__init__(self, algorithm='RRTConnect',
                robot_checker_factory=robot_checker_factory)

        self.multiplier = multiplier
        self.fraction = fraction


    """ Computes the 'range' parameter from DOF resolutions.

    The value is calculated from the active DOFs of 'robot' such that there
    will be an fixed number of state validity checks per extension. That number
    is configured by the 'multiplier' or 'fraction' constructor arguments.

    @param robot with active DOFs set
    @return value of the range parameter
    """
    def ComputeRange(self, robot):
        epsilon = 1e-6

        # Compute the maximum DOF range, treating circle joints as SO(2).
        dof_limit_lower, dof_limit_upper = robot.GetActiveDOFLimits()
        dof_ranges = numpy.zeros(robot.GetActiveDOF())

        for index, dof_index in enumerate(robot.GetActiveDOFIndices()):
            joint = robot.GetJointFromDOFIndex(dof_index)
            axis_index = dof_index - joint.GetDOFIndex()

            if joint.IsCircular(axis_index):
                # There are 2*pi radians in a circular joint, but the maximum
                # distance betwen any two points in SO(2) is pi. This is the
                # definition used by OMPL.
                dof_ranges[index] = numpy.pi
            else:
                dof_ranges[index] = (dof_limit_upper[index]
                                   - dof_limit_lower[index])

        # Duplicate the logic in or_ompl to convert the DOF resolutions to
        # a fraction of the longest extent of the state space.
        maximum_extent = numpy.linalg.norm(dof_ranges)
        dof_resolution = robot.GetActiveDOFResolutions()
        conservative_resolution = numpy.min(dof_resolution)
        conservative_fraction = conservative_resolution / maximum_extent

        if self.multiplier is not None:
            multiplier = self.multiplier
        elif self.fraction is not None:
            multiplier = max(round(self.fraction / conservative_fraction), 1)
        else:
            raise ValueError('Either multiplier or fraction must be set.')

        # Then, scale this by the user-supplied multiple. Finally, subtract
        # a small value to cope with numerical precision issues inside OMPL.
        return multiplier * conservative_resolution - epsilon

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, ompl_args=None, **kw_args):
        if ompl_args is None:
            ompl_args = dict()

        ompl_args.setdefault('range', self.ComputeRange(robot))

        return self._Plan(robot, goal=goal, ompl_args=ompl_args, **kw_args)

    @ClonedPlanningMethod
    def PlanToTSR(self, robot, tsrchains, ompl_args=None, **kw_args):
        if ompl_args is None:
            ompl_args = dict()

        ompl_args.setdefault('range', self.ComputeRange(robot))

        return self._Plan(robot, tsrchains=tsrchains, ompl_args=ompl_args, **kw_args)


# Alias to maintain backwards compatability.
class RRTConnect(OMPLRangedPlanner):
    def __init__(self):
        super(RRTConnect, self).__init__(algorithm='RRTConnect', fraction=0.2)
        warnings.warn('RRTConnect has been replaced by OMPLRangedPlanner.',
                DeprecationWarning)


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

    @ClonedPlanningMethod
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

        # Tag the trajectory as non-deterministic since the OMPL path
        # simplifier samples random candidate shortcuts. It does not, however,
        # change the endpoint, so we leave DETERMINISTIC_ENDPOINT unchanged.
        SetTrajectoryTags(output_path, {
            Tags.SMOOTH: True,
            Tags.DETERMINISTIC_TRAJECTORY: False,
        }, append=True)

        return output_path
