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
import logging, numpy, openravepy
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

class NamedPlanner(BasePlanner):
    def __init__(self, delegate_planner=None):
        """
        @param delegate_planner planner used for PlanToConfiguration
        """
        super(NamedPlanner, self).__init__()
        self.delegate_planner = delegate_planner

    def __str__(self):
        return 'NamedPlanner'

    @PlanningMethod
    def PlanToNamedConfiguration(self, robot, name, **kw_args):
        """ Plan to a named configuration.

        The configuration is looked up by name in robot.configurations. Any
        DOFs not specified in the named configuration are ignored. Planning is
        performed by PlanToConfiguration using a delegate planner. If not
        specified, this defaults to robot.planner.

        @param name name of a saved configuration
        @param **kw_args optional arguments passed to PlanToConfiguration
        @returns traj trajectory
        """
        try:
            configurations = robot.configurations
        except AttributeError:
            raise PlanningError('{:s} does not have a table of named'
                                ' configurations.'.format(robot))

        try:
            saved_dof_indices, saved_dof_values = robot.configurations.get_configuration(name)
        except KeyError:
            raise PlanningError('{0:s} does not have named configuration "{1:s}".'.format(robot, name))

        arm_dof_indices = robot.GetActiveDOFIndices()
        arm_dof_values = robot.GetActiveDOFValues()

        # Extract the active DOF values from the configuration.
        for arm_dof_index, arm_dof_value in zip(saved_dof_indices, saved_dof_values):
            if arm_dof_index in arm_dof_indices:
                i = list(arm_dof_indices).index(arm_dof_index)
                arm_dof_values[i] = arm_dof_value

        # Delegate planning to another planner.
        planner = self.delegate_planner or robot.planner
        return planner.PlanToConfiguration(robot, arm_dof_values, **kw_args)

