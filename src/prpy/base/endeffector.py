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

import openravepy

class EndEffector(openravepy.Robot.Link):
    def __init__(self, manipulator):
        self.manipulator = manipulator

    def GetIndices(self):
        """Gets the DOF indicies associated with this end-effector.
        @return list of DOF indices
        """
        indices = self.manipulator.GetChildDOFIndices()
        indices.sort()
        return indices

    def GetDOFValues(self):
        """Gets the current DOF values of this end-effector.
        These DOF values correspond to the DOF indices returned by
        \ref GetIndices.
        @return list of DOF values
        """
        return self.manipulator.GetRobot().GetDOFValues(self.GetIndices())

    def SetDOFValues(self, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        """Sets the current DOF values of this end-effector.
        These DOF values correspond to the DOF indices returned by
        \ref GetIndices. Note that this method only changes the DOF values in
        OpenRAVE: it set a position set-point on the real robot.
        @param dof_values DOF values
        @param limits_action whether to enforce joint limits
        """
        self.manipulator.GetRobot().SetDOFValues(dof_values, self.GetIndices(), limits_action)

    def SetActive(self):
        """Sets this end-effector as active.
        This both: (1) sets the current manipulator as active and (2) sets the
        active DOF values to thosse associated with this end-effector.
        """
        self.GetRobot().SetActiveManipulator(self.manipulator)
        self.GetRobot().SetActiveDOFs(self.GetArmIndices())
