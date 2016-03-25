#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Siddhartha Srinivasa <siddh@cs.cmu.edu>
#          Michael Koval <mkoval@cs.cmu.edu>
#          Pras Velagapudi <mkoval@cs.cmu.edu>
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
from base import (BasePlanner,
                  PlanningError,
                  UnsupportedPlanningError,
                  ClonedPlanningMethod)


class OpenRAVEPlanner(BasePlanner):
    def __init__(self, algorithm='birrt'):
        super(OpenRAVEPlanner, self).__init__()

        self.setup = False
        self.algorithm = algorithm

        try:
            self.planner = openravepy.RaveCreatePlanner(self.env, algorithm)
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create {:s} module.'
                                           .format(str(self)))

    def __str__(self):
        return 'OpenRAVE {0:s}'.format(self.algorithm)

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a desired configuration with OpenRAVE. This will invoke the
        OpenRAVE planner specified in the OpenRAVEPlanner constructor.
        @param robot the robot whose active DOFs will be used
        @param goal the desired robot joint configuration
        @return traj a trajectory from current configuration to specified goal
        """

        return self._Plan(robot, goal, **kw_args)

    def _Plan(self, robot, goals, maxiter=500, continue_planner=False,
              or_args=None, **kw_args):

        # Get rid of default postprocessing
        extraParams = ('<_postprocessing planner="">'
                       '<_nmaxiterations>0</_nmaxiterations>'
                       '</_postprocessing>')
        # Maximum planner iterations
        extraParams += ('<_nmaxiterations>{:d}</_nmaxiterations>'
                        .format(maxiter))

        if or_args is not None:
            for key, value in or_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(k=str(key),
                                                             v=str(value))

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(goals)
        params.SetExtraParameters(extraParams)

        traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')

        try:
            self.env.Lock()

            # Plan.
            if (not continue_planner) or not self.setup:
                self.planner.InitPlan(robot, params)
                self.setup = True

            status = self.planner.PlanPath(traj, releasegil=True)
            from openravepy import PlannerStatus
            if status not in [PlannerStatus.HasSolution,
                              PlannerStatus.InterruptedWithSolution]:
                raise PlanningError('Planner returned with status {:s}.'
                                    .format(str(status)))
        except Exception as e:
            raise PlanningError('Planning failed with error: {:s}'.format(e))
        finally:
            self.env.Unlock()

        return traj


class BiRRTPlanner(OpenRAVEPlanner):

    def __init__(self):
        OpenRAVEPlanner.__init__(self, algorithm='birrt')

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        """
        Plan to a desired configuration with OpenRAVE. This will invoke the
        OpenRAVE planner specified in the OpenRAVEPlanner constructor.
        @param robot the robot whose active DOFs will be used
        @param goal the desired robot joint configuration
        @return traj a trajectory from current configuration to specified goal
        """
        return self._Plan(robot, goal, **kw_args)

    @ClonedPlanningMethod
    def PlanToConfigurations(self, robot, goals, **kw_args):
        """
        Plan to one of many configuration with OpenRAVE's BiRRT planner.
        @param robot the robot whose active DOFs will be used
        @param goals a list of desired robot joint configurations
        @return traj trajectory from current configuration to one of the goals
        """
        if len(goals[0]) != len(robot.GetActiveDOFIndices()):
            raise ValueError('Goals must be same length as robot active DOFs.')

        # Serialize list of goals into a single 1D vector
        # (This will raise ValueError if the goals are not equal length.)
        goals = numpy.ravel(numpy.vstack(goals))
        return self._Plan(robot, goals, **kw_args)
