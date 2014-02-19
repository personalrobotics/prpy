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

import copy, functools, numpy, openravepy
from .. import bind, clone, planning

class Manipulator(openravepy.Robot.Manipulator):
    def __init__(self):
        pass

    def __dir__(self):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__. 
        self = bind.InstanceDeduplicator.get_canonical(self)

        # Add planning methods to the tab-completion list.
        method_names = set(self.__dict__.keys())
        method_names.update(self.GetRobot().planner.get_planning_method_names())
        return list(method_names)

    def __getattr__(self, name):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__. 
        self = bind.InstanceDeduplicator.get_canonical(self)
        delegate_method = getattr(self.GetRobot().planner, name)

        # Resolve planner calls through the robot.planner field.
        # FIXME: We need to replicate the _PlanWrapper functionality here.
        if self.GetRobot().planner.is_planning_method(name):
            @functools.wraps(delegate_method)
            def wrapper_method(*args, **kw_args):
                return self._PlanWrapper(delegate_method, args, kw_args) 

            return wrapper_method

        raise AttributeError('{0:s} is missing method "{1:s}".'.format(repr(self), name))

    def CloneBindings(self, parent):
        self.__init__(self)

    def GetIndices(self):
        return self.GetArmIndices()

    def GetDOFValues(self):
        return self.GetRobot().GetDOFValues(self.GetIndices())

    def SetDOFValues(self, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        self.GetRobot().SetDOFValues(dof_values, self.GetIndices(), limits_action)

    def SetActive(self):
        self.GetRobot().SetActiveManipulator(self)
        self.GetRobot().SetActiveDOFs(self.GetArmIndices())

    def SetVelocityLimits(self, velocity_limits, min_accel_time):
        velocity_limits = numpy.array(velocity_limits, dtype='float')
        active_indices = self.GetIndices()

        # Set the velocity limits.
        or_velocity_limits = self.GetRobot().GetDOFVelocityLimits()
        or_velocity_limits[active_indices] = velocity_limits
        self.GetRobot().SetDOFVelocityLimits(or_velocity_limits)

        # Set the acceleration limits.
        or_accel_limits = self.GetRobot().GetDOFAccelerationLimits()
        or_accel_limits[active_indices] = velocity_limits / min_accel_time
        self.GetRobot().SetDOFAccelerationLimits(or_accel_limits)

    def _PlanWrapper(self, planning_method, args, kw_args):
        from prpy.clone import Clone, Cloned
        robot = self.GetRobot()
        with Clone(robot.GetEnv()):
            Cloned(self).SetActive()
            cloned_args = copy.copy(kw_args)
            cloned_args['execute'] = False
            cloned_traj = Cloned(robot)._PlanWrapper(planning_method, args, cloned_args)

            # Strip inactive DOFs from the trajectory.
            config_spec = Cloned(robot).GetActiveConfigurationSpecification()
            openravepy.planningutils.ConvertTrajectorySpecification(cloned_traj, config_spec)
            traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), cloned_traj.GetXMLId())
            traj.Clone(cloned_traj, 0)

        # Optionally execute the trajectory.
        if 'execute' not in kw_args or kw_args['execute']:
            return self.GetRobot().ExecuteTrajectory(traj, **kw_args)
        else:
            return traj
