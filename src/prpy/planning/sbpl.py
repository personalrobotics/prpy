#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Evan Shapiro <eashapir@andrew.cmu.edu>
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

from ..util import SetTrajectoryTags
from base import BasePlanner, PlanningError, PlanningMethod, UnsupportedPlanningError, Tags
import openravepy

class SBPLPlanner(BasePlanner):
    def __init__(self):
        super(SBPLPlanner, self).__init__()
        
        try:
            self.planner = openravepy.RaveCreatePlanner(self.env, 'SBPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create SBPL module')

    def setupEnv(self, env):
        self.env = env
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'SBPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create SBPL module.')

    def __str__(self):
        return 'SBPL'

    def SetPlannerParameters(self, params_yaml):
        self.planner_params = params_yaml

    @PlanningMethod
    def PlanToBasePose(self, robot, goal_pose, timelimit=60.0, return_first=False, **kw_args):
        """
        Plan to a base pose using SBPL
        @param robot
        @param goal_pose desired base pose
        @param timelimit timeout in seconds
        @param return_first return the first path found (if true, the planner will run until a path is found, ignoring the time limit)
        """
        params = openravepy.Planner.PlannerParameters()

        from openravepy import DOFAffine
        robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
        params.SetRobotActiveJoints(robot)

        config_spec = openravepy.RaveGetAffineConfigurationSpecification(DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, robot)
        #params.SetConfigurationSpecification(self.env, config_spec) # This breaks
        
        goal_config = openravepy.RaveGetAffineDOFValuesFromTransform(goal_pose, 
                                                                     DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)

        params.SetGoalConfig(goal_config)
        
        # Setup default extra parameters
        extra_params = self.planner_params

        limits = robot.GetAffineTranslationLimits();
        extents = [limits[0][0], limits[1][0], limits[0][1], limits[1][1]];
        extra_params["extents"] = extents

        extra_params["timelimit"] = timelimit
        if return_first:
            extra_params["return_first"] = 1
        else:
            extra_params["return_first"] = 0

        extra_params["initial_eps"] = 1.0

        for key, value in kw_args.iteritems():
            extra_params[key] = value

        params.SetExtraParameters(str(extra_params))
        traj = openravepy.RaveCreateTrajectory(self.env, '')
 
        try:
            self.planner.InitPlan(robot, params)
            status = self.planner.PlanPath(traj, releasegil=True)
            
        except Exception as e:
            raise PlanningError('Planning failed with error: {0:s}'.format(e))

        from openravepy import PlannerStatus
        if status not in [ PlannerStatus.HasSolution, PlannerStatus.InterruptedWithSolution ]:
            raise PlanningError('Planner returned with status {0:s}.'.format(str(status)))

        SetTrajectoryTags(traj, {Tags.SMOOTH: True}, append=True)
        return traj  
        
