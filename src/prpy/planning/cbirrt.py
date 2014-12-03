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

import logging, numpy, openravepy, os, tempfile
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod
import prpy.kin, prpy.tsr

class CBiRRTPlanner(BasePlanner):
    def __init__(self):
        super(CBiRRTPlanner, self).__init__()
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'CBiRRT')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create CBiRRT module.')
    
    def setupEnv(self, env):
        self.env = env
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'CBiRRT')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create CBiRRT module.')

    def __str__(self):
        return 'CBiRRT'

    def Plan(self, robot, smoothingitrs=None, timelimit=None, allowlimadj=0,
             start_config=None, extra_args=None, **kw_args):
        # Take a snapshot of the source environment for planning.
        self.env.LoadProblem(self.problem, robot.GetName())


        args = [ 'RunCBiRRT' ]
        if extra_args is not None:
            args += extra_args
        if smoothingitrs is not None:
            args += [ 'smoothingitrs', str(smoothingitrs) ]
        if timelimit is not None:
            args += [ 'timelimit', str(timelimit) ]
        if allowlimadj is not None:
            args += [ 'allowlimadj', str(int(allowlimadj)) ]
        if start_config is not None:
            args += [ 'jointstarts', str(len(start_config)), ' '.join([ str(x) for x in start_config ]) ]

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = 'cmovetraj.txt'
        args += [ 'filename', traj_path ]
        args_str = ' '.join(args)

        response = self.problem.SendCommand(args_str, True)
        if not response.strip().startswith('1'):
            raise PlanningError('Unknown error: ' + response)
         

        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            traj.deserialize(traj_xml)

        return traj

    def PlanToGoals(self, robot, goals, **kw_args):
        """
        Helper method to allow PlanToConfiguration and PlanToConfigurations to use same code
        without going back through PlanWrapper logic.
        @param robot 
        @param goals A list of goal configurations
        """
        extra_args = list()
        for goal in goals:
            goal_array = numpy.array(goal)
            
            if len(goal_array) != robot.GetActiveDOF():
                logging.error('Incorrect number of DOFs in goal configuration; expected {0:d}, got {1:d}'.format(
                        robot.GetActiveDOF(), len(goal_array)))
                raise PlanningError('Incorrect number of DOFs in goal configuration.')
            

            extra_args += [ 'jointgoals',  str(len(goal_array)), ' '.join([ str(x) for x in goal_array ]) ]
        return self.Plan(robot, extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToConfigurations(self, robot, goals, **kw_args):
        """
        Plan to a configuraiton wtih multi-goal CBiRRT. This adds each goal in goals
        as a goal for the planner and returns path that achieves one of the goals.
        @param robot
        @param goals A list of goal configurations
        """
        return self.PlanToGoals(robot, goals, **kw_args)


    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a single configuration with single-goal CBiRRT.
        @param robot
        @param goal goal configuration
        """
        return self.PlanToGoals(robot, [goal], **kw_args)

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, psample=0.1, **kw_args):
        """
        Plan to a desired end-effector pose.
        @param robot
        @param goal desired end-effector pose
        @param psample probability of sampling a goal
        @return traj
        """
        manipulator_index = robot.GetActiveManipulatorIndex()
        goal_tsr = prpy.tsr.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        tsr_chain = prpy.tsr.tsr.TSRChain(sample_goal=True, TSR=goal_tsr)

        extra_args  = [ 'TSRChain', tsr_chain.serialize() ]
        extra_args += [ 'psample', str(psample) ]
        return self.Plan(robot, extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                timelimit=5.0, smoothingitrs=250, **kw_args):
        """
        Plan to a desired end-effector offset.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param timelimit timeout in seconds
        @param smoothingitrs number of smoothing iterations
        @return traj
        """
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
            traj_tsr_chain = prpy.tsr.tsr.TSRChain(constrain=True, TSRs=[ trajtsr ])
        
        extra_args = ['psample', '0.1']
        extra_args += [ 'TSRChain', goal_tsr_chain.serialize() ]
        extra_args += [ 'TSRChain', traj_tsr_chain.serialize() ]
        return self.Plan(robot, allowlimadj=True, timelimit=timelimit,
                         smoothingitrs=smoothingitrs,
                         extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        """
        Plan to a goal TSR.
        @param robot
        @param tsrchains goal TSR chain
        @return traj
        """
        extra_args = list()
        
        use_psample = False
        for chain in tsrchains:
            extra_args += [ 'TSRChain', chain.serialize() ]
            if chain.sample_start or chain.sample_goal:
                use_psample = True

        if use_psample:
            extra_args += ['psample', '0.1']

        return self.Plan(robot, extra_args=extra_args, **kw_args)
