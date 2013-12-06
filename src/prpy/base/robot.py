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

import openravepy, numpy
from .. import named_config, planning, util
from prpy.clone import Clone, Cloned

class Robot(openravepy.Robot):
    def __init__(self):
        self.planner = None
        self.controllers = list()
        self.manipulators = list()
        self.configurations = named_config.ConfigurationLibrary()
        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        # Standard, commonly-used OpenRAVE plugins.
        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)

        # Automatically bind planning methods on this object.
        planning.Planner.bind(self, lambda: self.planner, executer=self._PlanWrapper)

    def CloneBindings(self, parent):
        self.planner = parent.planner
        self.controllers = list()
        self.manipulators = [ Cloned(manipulator) for manipulator in parent.manipulators ]
        self.configurations = parent.configurations
        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)

        planning.Planner.bind(self, lambda: self.planner, executer=self._PlanWrapper)

    def AttachController(self, name, args, dof_indices, affine_dofs, simulated):
        """
        Create and attach a controller to a subset of this robot's DOFs. If
        simulated is False, a controller is created using 'args' and is attached
        to the multicontroller. In simulation mode an IdealController is
        created instead. Regardless of the simulation mode, the multicontroller
        must be finalized before use.  @param name user-readable name used to identify this controller
        @param args real controller arguments
        @param dof_indices controlled joint DOFs
        @param affine_dofs controleld affine DOFs
        @param simulated simulation mode
        @returns created controller
        """
        if simulated:
            args = 'IdealController'

        delegate_controller = openravepy.RaveCreateController(self.GetEnv(), args)
        if delegate_controller is None:
            type_name = args.split()[0]
            message = 'Creating controller {0:s} of type {1:s} failed.'.format(name, type_name)
            raise openravepy.openrave_exception(message)

        self.multicontroller.AttachController(delegate_controller, dof_indices, affine_dofs)
                
        return delegate_controller

    def GetTrajectoryManipulators(self, traj):
        """
        Extract the manipulators that are active in a trajectory. A manipulator
        is considered active if joint values are specified for one or more of its
        controlled DOFs.
        @param traj input trajectory
        @returns list of active manipulators
        """
        traj_indices = set(util.GetTrajectoryIndices(traj))

        active_manipulators = []
        for manipulator in self.GetManipulators():
            manipulator_indices = set(manipulator.GetArmIndices())
            if traj_indices & manipulator_indices:
                active_manipulators.append(manipulator)

        return active_manipulators

    def RetimeTrajectory(self, traj, **kw_args):
        """
        Compute timing information for a trajectory, populating the
        trajectory's deltatime group. Timing information is necessary for
        successful execution in simulation.
        @param traj input trajectory
        @returns timed output trajectory
        """
        openravepy.planningutils.RetimeTrajectory(traj)
        return traj

    def ExecuteTrajectory(self, traj, retime=True, timeout=None, **kw_args):
        """
        Executes a trajectory and optionally waits for it to finish.
        @param traj input trajectory
        @param retime optionally retime the trajectory before executing it
        @param timeout duration to wait for execution
        @returns final executed trajectory
        """
        if retime:
            traj = self.RetimeTrajectory(traj)

        self.GetController().SetPath(traj)
        active_manipulators = self.GetTrajectoryManipulators(traj)
        active_controllers = [ manipulator.controller for manipulator in active_manipulators ]
        util.WaitForControllers(active_controllers, timeout=timeout)
        return traj

    def PlanToNamedConfiguration(self, name, execute=True, **kw_args):
        """
        Plan to a saved configuration stored in robot.configurations.
        @param name name of a saved configuration
        @param **kw_args optional arguments passed to PlanToConfiguration
        @returns traj trajectory
        """
        dof_indices, dof_values = self.configurations.get_configuration(name)

        with Clone(self.GetEnv()):
            Cloned(self).SetActiveDOFs(dof_indices)
            traj = Cloned(self).PlanToConfiguration(dof_values, execute=False, **kw_args)

            # Clone the trajectory back into the live environment
            live_traj = openravepy.RaveCreateTrajectory(self.GetEnv(), '')
            live_traj.Clone(traj, 0)

        if execute:
            return self.ExecuteTrajectory(live_traj, **kw_args)
        else:
            return live_traj

    def _PlanWrapper(self, planning_method, args, kw_args):
        # Call the planner.
        traj = planning_method(self, *args, **kw_args)

        # Strip inactive DOFs from the trajectory.
        config_spec = self.GetActiveConfigurationSpecification()
        openravepy.planningutils.ConvertTrajectorySpecification(traj, config_spec)

        # Optionally execute the trajectory.
        if 'execute' not in kw_args or kw_args['execute']:
            return self.ExecuteTrajectory(traj, **kw_args)
        else:
            return traj
