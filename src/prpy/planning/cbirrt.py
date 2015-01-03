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

import copy, logging, numpy, openravepy, os, tempfile
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod
import prpy.kin, prpy.tsr

class CBiRRTPlanner(BasePlanner):
    def __init__(self):
        super(CBiRRTPlanner, self).__init__()
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'CBiRRT')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create CBiRRT module.')

    def __str__(self):
        return 'CBiRRT'

    @PlanningMethod
    def PlanToConfigurations(self, robot, goals, **kw_args):
        """
        Plan to multiple goal configurations with CBiRRT. This adds each goal
        in goals as a root node in the tree and returns a path that reaches one
        of the goals.
        @param robot
        @param goals list of goal configurations
        @return traj output path
        """
        return self.Plan(robot, jointgoals=goals, smoothingitrs=0, **kw_args)


    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a single goal configuration with CBiRRT.
        @param robot
        @param goal goal configuration
        @return traj output path
        """
        return self.Plan(robot, jointgoals=[goal], smoothingitrs=0, **kw_args)

    @PlanningMethod
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

        return self.Plan(robot, tsr_chains=[tsr_chain], psample=0.1,
                         smoothingitrs=0, **kw_args)

    @PlanningMethod
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
        if direction.shape != (3,):
            raise ValueError('Direction must be a three-dimensional vector.')
        if not (distance >= 0):
            raise ValueError('Distance must be non-negative; got {:f}.'.format(
                             distance))

        with robot:
            manip = robot.GetActiveManipulator()
            H_world_ee = manip.GetEndEffectorTransform()

            # 'object frame w' is at ee, z pointed along direction to move
            H_world_w = prpy.kin.H_from_op_diff(H_world_ee[0:3,3], direction)
            H_w_ee = numpy.dot(prpy.kin.invert_H(H_world_w), H_world_ee)
        
            # Serialize TSR string (goal)
            Hw_end = numpy.eye(4)
            Hw_end[2,3] = distance

            goaltsr = prpy.tsr.tsr.TSR(T0_w = numpy.dot(H_world_w,Hw_end), 
                                     Tw_e = H_w_ee, 
                                     Bw = numpy.zeros((6,2)), 
                                     manip = robot.GetActiveManipulatorIndex())
            goal_tsr_chain = prpy.tsr.tsr.TSRChain(sample_goal = True,
                                                 TSRs = [goaltsr])
            # Serialize TSR string (whole-trajectory constraint)
            Bw = numpy.zeros((6,2))
            epsilon = 0.001
            Bw = numpy.array([[-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [min(0.0, distance),  max(0.0, distance)],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon]])

            trajtsr = prpy.tsr.tsr.TSR(T0_w = H_world_w, 
                                   Tw_e = H_w_ee, 
                                   Bw = Bw, 
                                   manip = robot.GetActiveManipulatorIndex())
            traj_tsr_chain = prpy.tsr.tsr.TSRChain(constrain=True, TSRs=[trajtsr])
        
        return self.Plan(robot,
            psample=0.1,
            tsr_chains=[goal_tsr_chain, traj_tsr_chain],
            # Smooth since this is a constrained trajectory.
            smoothingitrs=smoothingitrs
            **kw_args
        )

    @PlanningMethod
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
        psample = None
        is_constrained = False

        for chain in tsr_chains:
            if chain.sample_start or chain.sample_goal:
                psample = 0.1

            if chain.constrian:
                is_constrained = True

        # Only smooth constrained trajectories.
        if not is_constrained:
            smoothingitrs = 0

        return self.Plan(robot,
            psample=psample,
            smoothingitrs=smoothingitrs,
            tsr_chains=tsr_chains,
            **kw_args
        )

    def Plan(self, robot, smoothingitrs=None, timelimit=None, allowlimadj=0,
             jointstarts=None, jointgoals=None, psample=None, tsr_chains=None,
             extra_args=None, **kw_args):
        self.env.LoadProblem(self.problem, robot.GetName())

        args = [ 'RunCBiRRT' ]

        if extra_args is not None:
            args += extra_args

        if smoothingitrs is not None:
            if smoothingitrs < 0:
                raise ValueError('Invalid number of smoothing iterations. Value'
                                 'must be non-negative; got  {:d}.'.format(
                                    smoothingitrs))

            args += [ 'smoothingitrs', str(smoothingitrs) ]

        if timelimit is not None:
            if not (timelimit > 0):
                raise ValueError('Invalid value for "timelimit". Limit must be'
                                 ' non-negative; got {:f}.'.format(timelimit))

            args += [ 'timelimit', str(timelimit) ]

        if allowlimadj is not None:
            args += [ 'allowlimadj', str(int(allowlimadj)) ]

        if psample is not None:
            if not (0 <= psample <= 1):
                raise ValueError('Invalid value for "psample". Value must be in'
                                 ' the range [0, 1]; got {:f}.'.format(psample))

            args += [ 'psample', str(psample) ]

        if jointstarts is not None:
            for start_config in jointstarts:
                if len(start_config) != robot.GetActiveDOF():
                    raise ValueError(
                        'Incorrect number of DOFs in start configuration;'
                        ' expected {:d}, got {:d}'.format(
                            robot.GetActiveDOF(), len(start_config)
                        )
                    )
                
                args += ['jointstarts'] + self.serialize_dof_values(start_config)

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

        if tsr_chains is not None:
            for tsr_chain in tsr_chains:
                args += [ 'TSRChain', tsr_chain.serialize() ]

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = 'cmovetraj.txt'
        args += [ 'filename', traj_path ]
        args_str = ' '.join(args)

        response = self.problem.SendCommand(args_str, True)
        if not response.strip().startswith('1'):
            raise PlanningError('Unknown error: ' + response)
         
        # Construct the output trajectory.
        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')
            traj.deserialize(traj_xml)

        # Strip extraneous groups from the output trajectory.
        # TODO: Where are these groups coming from!?
        cspec = robot.GetActiveConfigurationSpecification()
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)

        return traj

    @staticmethod
    def serialize_dof_values(dof_values):
        return [ str(len(dof_values)), 
                 ' '.join([ str(x) for x in dof_values]) ]

