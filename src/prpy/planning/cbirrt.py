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
from base import (BasePlanner, PlanningError, UnsupportedPlanningError,
                  ClonedPlanningMethod, LockedPlanningMethod, Tags)
import prpy.kin
import prpy.tsr


class CBiRRTPlanner(BasePlanner):
    def __init__(self):
        super(CBiRRTPlanner, self).__init__()

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
        goal_tsr = prpy.tsr.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        tsr_chain = prpy.tsr.tsr.TSRChain(sample_goal=True, TSR=goal_tsr)

        kw_args.setdefault('psample', 0.1)
        kw_args.setdefault('smoothingitrs', 0)

        return self.Plan(robot, tsr_chains=[tsr_chain], **kw_args)

    @LockedPlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                smoothingitrs=100, **kw_args):
        """
        Plan to a desired end-effector offset.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
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
            H_world_w = prpy.kin.H_from_op_diff(H_world_ee[0:3, 3], direction)
            H_w_ee = numpy.dot(prpy.kin.invert_H(H_world_w), H_world_ee)

            # Serialize TSR string (goal)
            Hw_end = numpy.eye(4)
            Hw_end[2, 3] = distance

            goaltsr = prpy.tsr.tsr.TSR(T0_w=numpy.dot(H_world_w, Hw_end),
                                       Tw_e=H_w_ee,
                                       Bw=numpy.zeros((6, 2)),
                                       manip=robot.GetActiveManipulatorIndex())
            goal_tsr_chain = prpy.tsr.tsr.TSRChain(sample_goal=True,
                                                   TSRs=[goaltsr])
            # Serialize TSR string (whole-trajectory constraint)
            Bw = numpy.zeros((6, 2))
            epsilon = 0.001
            Bw = numpy.array([[-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [min(0.0, distance),  max(0.0, distance)],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon]])

            traj_tsr = prpy.tsr.tsr.TSR(
                T0_w=H_world_w, Tw_e=H_w_ee, Bw=Bw,
                manip=robot.GetActiveManipulatorIndex())
            traj_tsr_chain = prpy.tsr.tsr.TSRChain(constrain=True,
                                                   TSRs=[traj_tsr])

        kw_args.setdefault('psample', 0.1)

        return self.Plan(
            robot,
            tsr_chains=[goal_tsr_chain, traj_tsr_chain],
            # Smooth since this is a constrained trajectory.
            smoothingitrs=smoothingitrs,
            **kw_args
        )

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

        # if seed is provided, set seed for each tsrchain
        # TODO: This doesn't have any effect, it will get lost in serialization.
        if kw_args and 'seed' in kw_args:
            map(lambda t: t.set_seed(kw_args['seed']), tsr_chains)

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
             extra_args=None, **kw_args):
        from openravepy import CollisionOptions, CollisionOptionsStateSaver

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

        args = ['RunCBiRRT']

        # By default, CBiRRT interprets the DOF resolutions as an
        # L-infinity norm; this flag turns on the L-2 norm instead.
        args += ['bdofresl2norm', '1']
        args += ['steplength', '0.05999']

        if extra_args is not None:
            args += extra_args

        if smoothingitrs is not None:
            if smoothingitrs < 0:
                raise ValueError('Invalid number of smoothing iterations. '
                                 'Value must be non-negative; got  {:d}.'
                                 .format(smoothingitrs))

            args += ['smoothingitrs', str(smoothingitrs)]

        if timelimit is not None:
            if not (timelimit > 0):
                raise ValueError('Invalid value for "timelimit". Limit must be'
                                 ' non-negative; got {:f}.'.format(timelimit))

            args += ['timelimit', str(timelimit)]

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

        if jointgoals is not None:
            for goal_config in jointgoals:
                if len(goal_config) != robot.GetActiveDOF():
                    raise ValueError(
                        'Incorrect number of DOFs in goal configuration;'
                        ' expected {:d}, got {:d}'.format(
                            robot.GetActiveDOF(), len(goal_config)
                        )
                    )

                args += ['jointgoals'] + self.serialize_dof_values(goal_config)

            if len(jointgoals) > 1:
                is_endpoint_deterministic = False

        if tsr_chains is not None:
            for tsr_chain in tsr_chains:
                args += ['TSRChain', SerializeTSRChain(tsr_chain)]

                if tsr_chain.sample_goal:
                    is_endpoint_deterministic = False
                if tsr_chain.constrain:
                    is_constrained = True

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = 'cmovetraj.txt'
        args += ['filename', traj_path]
        args_str = ' '.join(args)

        with CollisionOptionsStateSaver(env.GetCollisionChecker(),
                                        CollisionOptions.ActiveDOFs):
            response = problem.SendCommand(args_str, True)

        if not response.strip().startswith('1'):
            raise PlanningError('Unknown error: ' + response,
                deterministic=False)

        # Construct the output trajectory.
        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(
                env, 'GenericTrajectory')
            traj.deserialize(traj_xml)

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
    return outstring
