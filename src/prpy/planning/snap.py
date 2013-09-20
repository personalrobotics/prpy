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
import logging, openravepy
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

class SnapPlanner(BasePlanner):
    def __init__(self):
        self.env = openravepy.Environment()
 
    def __str__(self):
        return 'snap'

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, snap_tolerance=0.1, **kw_args):
        active_indices = robot.GetActiveDOFIndices()
        current_dof_values = robot.GetActiveDOFValues()

        # Only snap if we're close to the goal configuration.
        if (goal - current_dof_values).max() > snap_tolerance:
            raise UnsupportedPlanningError('Distance from goal larger than snap tolerance. Failing planning.')

        # Create a two-point trajectory that takes us to the goal.
        logging.info('Snapping to goal configuration with a straight line trajectory.')
        traj = openravepy.RaveCreateTrajectory(self.env, '')
        manip = robot.GetActiveManipulator()
        traj.Init(manip.GetArmConfigurationSpecification())
        traj.Insert(0, current_dof_values)
        traj.Insert(1, goal)
        return traj


    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, **kw_args):
        raise UnsupportedPlanningError('PlanToEndEffectorOffset not implemented for snap planner')

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, **kw_args):
        raise UnsupportedPlanningError('PlanToEndEffectorPose not implemented for snap planner')

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        raise UnsupportedPlanningError('PlanToTSR not implemented for snap planner')
    
