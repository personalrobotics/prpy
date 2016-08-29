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
        params.SetInitialConfig(robot.GetActiveDOFValues())

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
    include an integer number 'multipler' of state validity checks.

    @param algorithm name of the OMPL planner to create
    @param robot_checker_factory robot collision checker factory
    @param multiplier number of state validity checks per extension
    """
    def __init__(self, algorithm='RRTConnect', robot_checker_factory=None,
            multiplier=1):
        if multiplier < 1 or not isinstance(multiplier, int):
            raise ValueError('Multiplier must be a positive integer.')

        OMPLPlanner.__init__(self, algorithm='RRTConnect',
                robot_checker_factory=robot_checker_factory)
        self.multiplier = multiplier
        self.epsilon = 1e-6


    """ Set a default 'range' parameter from DOF resolutions.

    This function returns a copy of the input 'ompl_args' argument with the
    'range' parameter set to a default value, if it was not already set. The
    default value is calculated from the active DOFs of 'robot such that there
    will be an integer number of state validity checks per extension. That
    multiplier is configured in the constructor of this class.

    @param robot with active DOFs set
    @param ompl_args arguments to append to, defaults to an empty dictionary
    @return copy of ompl_args with 'range' set to the default value
    """
    def _SetPlannerRange(self, robot, ompl_args=None):
        from copy import deepcopy

        if ompl_args is None:
            ompl_args = dict()
        else:
            ompl_args = deepcopy(ompl_args)

        # Set the default range to the collision checking resolution.
        if 'range' not in ompl_args:
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
            ompl_range  = conservative_ratio * longest_extent - self.epsilon
            ompl_range *= self.multiplier

            ompl_args['range'] = ompl_range
            logger.debug('Defaulted RRT-Connect range parameter to %.3f.',
                         ompl_range)

        return ompl_args

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, ompl_args=None, **kw_args):
        ompl_args = self._SetPlannerRange(robot, ompl_args=ompl_args)
        return self._Plan(robot, goal=goal, ompl_args=ompl_args, **kw_args)

    @ClonedPlanningMethod
    def PlanToTSR(self, robot, tsrchains, ompl_args=None, **kw_args):
        ompl_args = self._SetPlannerRange(robot, ompl_args=ompl_args)
        return self._Plan(robot, tsrchains=tsrchains, ompl_args=ompl_args, **kw_args)


# Alias to maintain backwards compatability.
class RRTConnect(OMPLRangedPlanner):
    def __init__(self):
        super(RRTConnect, self).__init__(algorithm='RRTConnect')
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

class ConstrainedOMPLPlanner(OMPLPlanner):
    def __init__(self):
        OMPLPlanner.__init__(self, algorithm='CRRTConnect')

    @ClonedPlanningMethod
    def PlanToTSR(self, robot, tsrchains, planner_range=0.1, ompl_args=None, **kw_args):
        """
        Plan using the given TSR chains with OMPL. 
        @param robot
        @param tsrchains A list of TSRChain objects to respect during planning
        @param planner_range the max stepsize for the planner before a projection
         is applied
        @param ompl_args ompl RRTConnect specific parameters
        @return traj
        """
        if ompl_args is None:
            ompl_args = {}
        ompl_args['range'] = planner_range
        return self._TSRPlan(robot, tsrchains, ompl_args=ompl_args, **kw_args)

    @ClonedPlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, planner_range=0.1,
                                smoothingitrs=100, **kw_args):
        """
        Plan to a desired end-effector offset.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param planner_range the max stepsize for the planner before a projection
         is applied
        @param smoothingitrs number of smoothing iterations to run
        @return traj output path
        """
        direction = numpy.array(direction, dtype=float)

        if direction.shape != (3,):
            raise ValueError('Direction must be a three-dimensional vector.')
        if not (distance >= 0):
            raise ValueError('Distance must be non-negative; got {:f}.'.format(
                             distance))

        with robot:
            manip = robot.GetActiveManipulator()
            H_world_ee = manip.GetEndEffectorTransform()

            # 'object frame w' is at ee, z pointed along direction to move
            import prpy
            H_world_w = prpy.kin.H_from_op_diff(H_world_ee[0:3,3], direction)
            H_w_ee = numpy.dot(prpy.kin.invert_H(H_world_w), H_world_ee)

            # Serialize TSR string (whole-trajectory constraint)
            Bw = numpy.zeros((6,2))
            epsilon = 0.00
            Bw = numpy.array([[-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [min(-epsilon, distance-epsilon),  
                                max(epsilon, distance+epsilon)],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon]])

            from prpy.tsr.tsr import TSR, TSRChain
            trajtsr = TSR(T0_w = H_world_w, 
                          Tw_e = H_w_ee, 
                          Bw = Bw, 
                          manip = robot.GetActiveManipulatorIndex())

            traj_tsr_chain = TSRChain(constrain=True, TSRs=[trajtsr])

            # Compute a goal pose
            goal_pose = manip.GetEndEffectorTransform()
            goal_pose[:3,3] += direction*distance/numpy.linalg.norm(direction)

            # Get all IKs for the goal pose and send them all to the planner
            goal_configs = manip.FindIKSolutions(goal_pose, 0)
        if goal_configs is None:
            raise PlanningError('Failed to find an IK solution at the goal configuration.')

        goal = []
        for g in goal_configs:
            goal += g.tolist()

        ompl_args = {'range': planner_range}

        from prpy.viz import RenderTSRList
        return self._TSRPlan(robot, [traj_tsr_chain], goal=goal, ompl_args=ompl_args, **kw_args)

    def _TSRPlan(self, robot, tsrchains, **kw_args):
        extra_params = ''
        for chain in tsrchains:
            if chain.constrain:
                extra_params += '<{k:s}>{v:s}</{k:s}>'.format(k = 'tsr_chain_constraint',
                                                              v=chain.to_json())
        if len(extra_params) == 0:
            raise UnsupportedPlanningError('Only constrained TSR chains are supported by ConstrainedOMPLPlanner')
        return self._Plan(robot, formatted_extra_params=extra_params, **kw_args)
