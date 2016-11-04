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
from ..collision import (
    BakedRobotCollisionCheckerFactory,
    DefaultRobotCollisionCheckerFactory,
    SimpleRobotCollisionCheckerFactory,
)
from ..tsr import TSR, TSRChain
from ..util import SetTrajectoryTags
from .adapters import PlanToEndEffectorOffsetTSRAdapter
from .base import (
    Planner,
    LockedPlanningMethod,
    PlanningError,
    Tags,
    UnsupportedPlanningError,
    save_dof_limits
)
import contextlib


class CBiRRTPlanner(Planner):
    def __init__(self, robot_checker_factory=None, timelimit=5.):
        super(CBiRRTPlanner, self).__init__()

        self.timelimit = timelimit
        self.mimic_trajectories = {}

        if robot_checker_factory is None:
            robot_checker_factory = DefaultRobotCollisionCheckerFactory

        self.robot_checker_factory = robot_checker_factory
        if isinstance(robot_checker_factory, SimpleRobotCollisionCheckerFactory):
            self._is_baked = False
        elif isinstance(robot_checker_factory, BakedRobotCollisionCheckerFactory):
            self._is_baked = True
        else:
            raise NotImplementedError(
                'CBiRRT only supports Simple and BakedRobotCollisionChecker.')

    def __str__(self):
        return 'CBiRRT'

    @LockedPlanningMethod
    def PlanToConfigurations(self, robot, goals, **kw_args):
        """
        Plan to multiple goal configurations with CBiRRT. This adds each goal
        in goals as a root node in the tree and returns a path that reaches one
        of the goals.
        @param robot
        @param goals list of goal configurations
        @return traj output path
        """
        kw_args.setdefault('smoothingitrs', 0)
        return self.Plan(robot, jointgoals=goals, **kw_args)

    @LockedPlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a single goal configuration with CBiRRT.
        @param robot
        @param goal goal configuration
        @return traj output path
        """
        kw_args.setdefault('smoothingitrs', 0)
        return self.Plan(robot, jointgoals=[goal], **kw_args)

    @LockedPlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, **kw_args):
        """
        Plan to a desired end-effector pose.
        @param robot
        @param goal desired end-effector pose
        @param psample probability of sampling a goal
        @return traj output path
        """
        manipulator_index = robot.GetActiveManipulatorIndex()
        goal_tsr = TSR(T0_w=goal_pose, manip=manipulator_index)
        tsr_chain = TSRChain(sample_goal=True, TSR=goal_tsr)

        kw_args.setdefault('psample', 0.1)
        kw_args.setdefault('smoothingitrs', 0)

        return self.Plan(robot, tsr_chains=[tsr_chain], **kw_args)


    @LockedPlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, **kw_args):
        """
        Plan to a desired end-effector offset.

        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param smoothingitrs number of smoothing iterations to run
        @return traj output path
        """
        chains = PlanToEndEffectorOffsetTSRAdapter.CreateTSRChains(
                    robot, direction, distance)
        return self.PlanToTSR(robot, tsr_chains=chains, **kw_args)


    @LockedPlanningMethod
    def PlanToTSR(self, robot, tsr_chains, smoothingitrs=100, **kw_args):
        """
        Plan to a goal specified as a list of TSR chains. CBiRRT supports an
        arbitrary list of start, goal, and/or constraint TSR chains. The path
        will be smoothed internally by CBiRRT if one or more constraint TSR
        chains are specified.
        @param robot
        @param tsr_chains list of TSR chains
        @param smoothingitrs number of smoothing iterations to run
        @return traj output path
        """
        is_constrained = False

        for chain in tsr_chains:
            if chain.sample_start or chain.sample_goal:
                kw_args.setdefault('psample', 0.1)

            if chain.constrain:
                is_constrained = True

        # Only smooth constrained trajectories.
        if not is_constrained:
            smoothingitrs = 0

        return self.Plan(
            robot,
            smoothingitrs=smoothingitrs,
            tsr_chains=tsr_chains,
            **kw_args
        )

    def Plan(self, robot, smoothingitrs=None, timelimit=None, allowlimadj=0,
             jointstarts=None, jointgoals=None, psample=None, tsr_chains=None,
             extra_args=None, save_mimic_trajectories=False, **kw_args):
        """
        @param allowlimadj If True, adjust the joint limits to include
            the robot's start configuration
        """
        from openravepy import CollisionOptionsStateSaver, Robot, KinBody

        if timelimit is None:
            timelimit = self.timelimit

        if timelimit <= 0.:
            raise ValueError('Invalid value for "timelimit". Limit must be'
                             ' non-negative; got {:f}.'.format(timelimit))

        env = robot.GetEnv()
        problem = openravepy.RaveCreateProblem(env, 'CBiRRT')

        if problem is None:
            raise UnsupportedPlanningError('Unable to create CBiRRT module.')

        is_endpoint_deterministic = True
        is_constrained = False

        # TODO We may need this work-around because CBiRRT doesn't like it
        # when an IK solver other than GeneralIK is loaded (e.g. nlopt_ik).
        # self.ClearIkSolver(robot.GetActiveManipulator())

        env.LoadProblem(problem, robot.GetName())

        args =  ['RunCBiRRT']
        args += ['timelimit', str(timelimit)]

        if self._is_baked:
            args += ['bbakedcheckers', '1']

        if extra_args is not None:
            args += extra_args

        if smoothingitrs is not None:
            if smoothingitrs < 0:
                raise ValueError('Invalid number of smoothing iterations. '
                                 'Value must be non-negative; got  {:d}.'
                                 .format(smoothingitrs))

            args += ['smoothingitrs', str(smoothingitrs)]

        if allowlimadj is not None:
            args += ['allowlimadj', str(int(allowlimadj))]

        if psample is not None:
            if not (0 <= psample <= 1):
                raise ValueError('Invalid value for "psample". Value must be '
                                 'in the range [0, 1]; got {:f}.'
                                 .format(psample))

            args += ['psample', str(psample)]

        if jointstarts is not None:
            for start_config in jointstarts:
                if len(start_config) != robot.GetActiveDOF():
                    raise ValueError(
                        'Incorrect number of DOFs in start configuration;'
                        ' expected {:d}, got {:d}'.format(
                            robot.GetActiveDOF(), len(start_config)
                        )
                    )
                args += (['jointstarts'] +
                         self.serialize_dof_values(start_config))
                
        #TODO need to change robot.GetActiveDOF() to something else
        # That can take into account another robot

        #FIXME Ordinally we perform this check. However it assumes that
        # we are only planning for the robot, which is not true
        # if we are planning a constrained task with an object
        # (ie opening a door). I'm unsure how to modify this check 
        # to account for that. 
        if jointgoals is not None:
            for goal_config in jointgoals:
                '''
                if len(goal_config) != robot.GetActiveDOF():
                    raise ValueError(
                        'Incorrect number of DOFs in goal configuration;'
                        ' expected {:d}, got {:d}'.format(
                            robot.GetActiveDOF(), len(goal_config)
                        )
                    )
                '''
                args += ['jointgoals'] + self.serialize_dof_values(goal_config)

                
            if len(jointgoals) > 1:
                is_endpoint_deterministic = False

        raw_input("Continue?")
        if tsr_chains is not None:
            for tsr_chain in tsr_chains:
                args += ['TSRChain', SerializeTSRChain(tsr_chain)]

                if tsr_chain.sample_goal:
                    is_endpoint_deterministic = False
                if tsr_chain.constrain:
                    is_constrained = True

        # By default, CBiRRT interprets the DOF resolutions as an
        # L-infinity norm; this flag turns on the L-2 norm instead.
        args += ['bdofresl2norm', '1']
        args += ['steplength', '0.05999']                

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = 'cmovetraj.txt'
        args += ['filename', traj_path]
        args_str = ' '.join(args)

        # Bypass the context manager since CBiRRT does its own baking in C++.
        collision_checker = self.robot_checker_factory(robot)
        options = collision_checker.collision_options
        
        if tsr_chains is not None:
            mimicbodies = [env.GetKinBody(chain.mimicbodyname) 
                           for chain in tsr_chains if chain.mimicbodyname is not 'NULL']
            mimicbody_savers = [
                mimicbody.CreateKinBodyStateSaver(KinBody.SaveParameters.LinkTransformation)
                for mimicbody in mimicbodies]
        else:
            mimicbody_savers = []

        with CollisionOptionsStateSaver(env.GetCollisionChecker(), options), \
            robot.CreateRobotStateSaver(Robot.SaveParameters.ActiveDOF | 
                                        Robot.SaveParameters.LinkTransformation), \
            contextlib.nested(*mimicbody_savers), save_dof_limits(robot):
            response = problem.SendCommand(args_str, True)

        with CollisionOptionsStateSaver(env.GetCollisionChecker(), options):
            response = problem.SendCommand(args_str, True)

        if not response.strip().startswith('1'):
            raise PlanningError('Unknown error: ' + response,
                deterministic=False)

        # Construct the output trajectory.
        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(env, '')
            traj.deserialize(traj_xml)

        # Mimic traj processing if requested
        if save_mimic_trajectories:
            from prpy.util import CopyTrajectory

            cspec = traj.GetConfigurationSpecification()
            traj_bodies = cspec.ExtractUsedBodies(robot.GetEnv())
          
            # Extract non-robot trajecotry
            self.mimic_trajectories = {}
            for body in traj_bodies:
                if body.GetName() == robot.GetName():
                    continue

                
                object_cspec = body.GetActiveConfigurationSpecification('GenericTrajectory')
                with open(traj_path, 'rb') as traj_file:
                    traj_xml = traj_file.read()
                    object_traj = openravepy.RaveCreateTrajectory(env, '')
                    object_traj.deserialize(traj_xml)
                openravepy.planningutils.ConvertTrajectorySpecification(object_traj, object_cspec)
                self.mimic_trajectories[body.GetName()] = object_traj

        # Tag the trajectory as non-determistic since CBiRRT is a randomized
        # planner. Additionally tag the goal as non-deterministic if CBiRRT
        # chose from a set of more than one goal configuration.
        SetTrajectoryTags(traj, {
            Tags.CONSTRAINED: is_constrained,
            Tags.DETERMINISTIC_TRAJECTORY: False,
            Tags.DETERMINISTIC_ENDPOINT: is_endpoint_deterministic,
        }, append=True)

        # Strip extraneous groups from the output trajectory.
        # TODO: Where are these groups coming from!?
        cspec = robot.GetActiveConfigurationSpecification('linear')
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)

        return traj

    def GetMimicPath(self, body_name, env=None):
        traj = self.mimic_trajectories.get(body_name, None)
        if traj is not None and env is not None:
            from prpy.util import CopyTrajectory
            traj = CopyTrajectory(traj, env=env)
        return traj

    def ClearIkSolver(self, manip):
        manip.SetIkSolver(None)
        manip.SetLocalToolTransform(manip.GetLocalToolTransform())
        manip.SetIkSolver(None)
        if manip.GetIkSolver() is not None:
            raise ValueError('Unable to clear IkSolver')

    @staticmethod
    def serialize_dof_values(dof_values):
        return [str(len(dof_values)),
                ' '.join([str(x) for x in dof_values])]


def SerializeTransform12Col(tm, format='%.5f'):
    return ' '.join([(format % (i,)) for i in tm[0:3, :].T.reshape(12)])


def SerializeArray(a, format='%.5f'):
    return ' '.join([(format % (i,)) for i in a.reshape(-1)])


def SerializeTSR(self):
    """
    Function for Serializing TSRs for CBIRRT.

    SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

    Input:
    manipindex (int): the 0-indexed index of the robot's manipulator
    bodyandlink (str): body and link which is used as the 0 frame. Format
                       'body_name link_name'. For world frame, specify 'NULL'
    T0_w (double 4x4): transform matrix of the TSR's reference frame relative
                       to the 0 frame
    Tw_e (double 4x4): transform matrix of the TSR's offset frame relative to
                       the w frame
    Bw (double 1x12): bounds in x y z roll pitch yaw.
                      Format: [x_min, x_max, y_min, y_max ...]

    Output:
    outstring (str): string to use for SerializeTSRChain function
    """
    return '%d %s %s %s %s' % (self.manipindex, self.bodyandlink,
                               SerializeTransform12Col(self.T0_w),
                               SerializeTransform12Col(self.Tw_e),
                               SerializeArray(self.Bw))


def SerializeTSRChain(self):
    """
    Function for Serializing TSR Chains for CBIRRT.

    _SerializeTSRChain(bSampleFromChain, bConstrainToChain,
                       numTSRs, allTSRstring,
                       mimicbodyname, mimicbodyjoints)

    Input:
    bSampleStartFromChain (0/1): 1: Use this chain for sampling start configs
                                 0: Ignore for sampling starts
    bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configs
                                0: Ignore for sampling goals
    bConstrainToChain (0/1): 1: Use this chain for constraining configs
                             0: Ignore for constraining
    numTSRs (int): Number of TSRs in this chain (must be > 0)
    allTSRstring (str): string of concatenated TSRs from SerializeTSR.
                        Should be like [TSRstring 1 ' ' TSRstring2 ...]
    mimicbodyname (str): name of associated mimicbody for this chain
                         (NULL if none associated)
    mimicbodyjoints (int [1xn]): 0-indexed indices of mimicbody's joints that
                                 are mimiced (INCREASING AND CONSECUTIVE)

    Output:
    outstring (str): string to include in call to cbirrt planner
    """
    allTSRstring = ' '.join([SerializeTSR(tsr) for tsr in self.TSRs])
    numTSRs = len(self.TSRs)
    outstring = '%d %d %d' % (int(self.sample_start),
                              int(self.sample_goal),
                              int(self.constrain))
    outstring += ' %d %s' % (numTSRs, allTSRstring)
    outstring += ' ' + self.mimicbodyname
    if len(self.mimicbodyjoints) > 0:
        outstring += ' %d %s' % (len(self.mimicbodyjoints),
                                 SerializeArray(self.mimicbodyjoints))
#        outstring += ' %s' % SerializeArray(self.mimicbodyjoints)
    return outstring
