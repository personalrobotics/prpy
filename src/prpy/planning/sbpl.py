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

from base import BasePlanner, PlanningMethod
import openravepy

class SBPLPlanner(BasePlanner):
    def __init__(self):
        super(SBPLPlanner, self).__init__()
        
        try:
            self.planner = openravepy.RaveCreatePlanner(self.env, 'SBPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create SBPL module')

    def __str__(self):
        return 'SBPL'


    @PlanningMethod
    def PlanToBasePose(self, robot, goal_pose, **kw_args):
        """
        Plan to a base pose using SBPL
        @param robot
        @param goal_pose desired base pose
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
        extra_params = ["cellsize", "0.1"]
        extra_params += ["numangles", "16"]


        limits = robot.GetAffineTranslationLimits();
        extents = [limits[0][0], limits[1][0], limits[0][1], limits[1][1]];
        extra_params += ["extents", " ".join([str(e) for e in extents])]
        extra_params += ["action", "1.0 0.0 0.1"]
        extra_params += ["action", "1.0 4.0 0.1"]
        extra_params += ["action", "1.0 -4.0 0.1"]
        extra_params += ["action", "0.0 4.0 0.1"]
        extra_params += ["action", "0.0 -4.0 0.1"]
        extra_params_str = ' '.join(extra_params)
        params.SetExtraParameters(extra_params_str)

        traj = openravepy.RaveCreateTrajectory(self.env, '')
 
        try:
            self.planner.InitPlan(robot, params)
            status = self.planner.PlanPath(traj, releasegil=True)
            
        except Exception as e:
            raise PlanningError('Planning failed with error: {0:s}'.format(e))

        from openravepy import PlannerStatus
        if status not in [ PlannerStatus.HasSolution, PlannerStatus.InterruptedWithSolution ]:
            raise PlanningError('Planner returned with status {0:s}.'.format(str(status)))

        return traj  
        
