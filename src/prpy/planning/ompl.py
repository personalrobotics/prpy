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

class OMPLPlanner(BasePlanner):
    def __init__(self, algorithm='RRTConnect'):
        super(OMPLPlanner, self).__init__()

        self.setup = False

        self.algorithm = algorithm
        try:
            self.planner = openravepy.RaveCreatePlanner(self.env, 'OMPL')
            self.simplifier = openravepy.RaveCreatePlanner(self.env, 'OMPLSimplifier')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create OMPL module.')

    def __str__(self):
        return 'OMPL {0:s}'.format(self.algorithm)

    def Plan(self, robot, params, shortcut_timeout=5.0, continue_planner=False, **kw_args):
        '''
        Helper function for running planner
        '''
        traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')

        try:
            self.env.Lock()

            # Plan.
            if (not continue_planner) or not self.setup:
                self.planner.InitPlan(robot, params)
                self.setup = True

            status = self.planner.PlanPath(traj, releasegil=True)
            from openravepy import PlannerStatus
            if status not in [ PlannerStatus.HasSolution,
                               PlannerStatus.InterruptedWithSolution ]:
                raise PlanningError('Planner returned with status {0:s}.'.format(
                                    str(status)))

            # Shortcut.
            params = openravepy.Planner.PlannerParameters()
            params.SetExtraParameters(
                '<time_limit>{:f}</time_limit>'.format(shortcut_timeout)
            )
            self.simplifier.InitPlan(robot, params)
            status = self.simplifier.PlanPath(traj, releasegil=True)
            if status not in [ PlannerStatus.HasSolution,
                               PlannerStatus.InterruptedWithSolution ]:
                raise PlanningError('Simplifier returned with status {0:s}.'.format(
                                    str(status)))
        except Exception as e:
            raise PlanningError('Planning failed with error: {0:s}'.format(e))
        finally:
            self.env.Unlock()

        return traj

    def TSRPlan(self, robot, tsrchains, timelimit=25.0, ompl_args = None, *args, **kw_args):
        """
        Helper function for planning with TSRs
        """
        extraParams = '<time_limit>{time_limit:f}</time_limit>'\
            '<planner_type>{algorithm:s}</planner_type>'.format(
                time_limit = timelimit,
                algorithm = self.algorithm
            )

        for chain in tsrchains:
            extraParams += '<{k:s}>{v:s}</{k:s}>'.format(k='tsr_chain', v=chain.serialize())

        if ompl_args is not None:
            for key,value in ompl_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(k = str(key), v = str(value))
                

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetExtraParameters(extraParams)

        return self.Plan(robot, params, *args, **kw_args)

        

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, timeout=25.0, shortcut_timeout=5.0, continue_planner=False, ompl_args = None, **kw_args):
        """
        Plan to a desired configuration with OMPL. This will invoke the OMPL
        planner specified in the OMPLPlanner constructor.
        @param robot
        @param goal desired configuration
        @return traj
        """
        extraParams = '<time_limit>{time_limit:f}</time_limit>'\
            '<planner_type>{algorithm:s}</planner_type>'.format(
                time_limit = timeout,
                algorithm = self.algorithm
            )

        if ompl_args is not None:
            for key,value in ompl_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(k = str(key), v = str(value))

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(goal)
        params.SetExtraParameters(extraParams)

        return self.Plan(robot, params, continue_planner=continue_planner, shortcut_timeout=shortcut_timeout)
        
    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, timelimit=25.0, shortcut_timeout=5.0, continue_planner=False, ompl_args = None, **kw_args):
        """
        Plan to a desired TSR set with OMPL. This will invoke the OMPL planner
        specified in the OMPLPlanner constructor and pass a list of TSR chains
        as the planning goal.

        @param robot
        @param tsrchains a list of TSR chains that define a goal set
        @return traj
        """
        return self.TSRPlan(robot, tsrchains, 
                            timeout=timelimit, ompl_args=ompl_args, 
                            shortcut_timeout=shortcut_timeout, 
                            continue_planner=continue_planner, 
                            **kw_args)

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, timelimit=25.0, ompl_args = None, **kw_args):
        """
        Plan to desired end-effector pose.
        @param robot 
        @param goal_pose desired end-effector pose
        @param timeout The maximum time to allow the plan to search for a solution
        @param ompl_args A dictionary of extra arguments to be passed to the ompl planner
        @return traj
        """
        manipulator_index = robot.GetActiveManipulatorIndex()
        from prpy.tsr.tsr import TSR, TSRChain
        goal_tsr = TSR(T0_w = goal_pose, manip = manipulator_index)
        tsr_chain = TSRChain(sample_goal = True, TSR = goal_tsr)
        return self.TSRPlan(robot, [tsr_chain], 
                            timeout=timelimit, ompl_args=ompl_args, 
                            **kw_args)
    '''        
    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, timelimit=5.0,
                                ompl_args=None, **kw_args):
        """
        Plan to a desired end-effector offset.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param timelimit timeout in seconds
        @param ompl_args A dictionary of extra arguments to be passed to the ompl planner
        @return traj
        """
        with robot:
            manip = robot.GetActiveManipulator()
            H_world_ee = manip.GetEndEffectorTransform()

            # 'object frame w' is at ee, z pointed along direction to move
            import prpy.kin
            H_world_w = prpy.kin.H_from_op_diff(H_world_ee[0:3,3], direction)
            H_w_ee = numpy.dot(prpy.kin.invert_H(H_world_w), H_world_ee)
        
            # Serialize TSR string (goal)
            Hw_end = numpy.eye(4)
            Hw_end[2,3] = distance

            from prpy.tsr.tsr import *
            goaltsr = TSR(T0_w = numpy.dot(H_world_w,Hw_end), 
                          Tw_e = H_w_ee, 
                          Bw = numpy.zeros((6,2)), 
                          manip = robot.GetActiveManipulatorIndex())
            goal_tsr_chain = TSRChain(sample_goal = True,
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

            trajtsr = TSR(T0_w = H_world_w, 
                          Tw_e = H_w_ee, 
                          Bw = Bw, 
                          manip = robot.GetActiveManipulatorIndex())
            traj_tsr_chain = TSRChain(constrain=True, TSRs=[ trajtsr ])

        return self.TSRPlan(robot, [goal_tsr_chain, traj_tsr_chain], 
                            timeout=timelimit, ompl_args=ompl_args, **kw_args)
        
     '''
