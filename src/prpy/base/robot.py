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

import functools, logging, openravepy, numpy
from .. import bind, named_config, planning, util
from prpy.clone import Clone, Cloned
from prpy.tsr.tsrlibrary import TSRLibrary

logger = logging.getLogger('robot')

class Robot(openravepy.Robot):
    def __init__(self, robot_name=None):
        self.planner = None

        try:
            self.tsrlibrary = TSRLibrary(self, robot_name=robot_name)
        except ValueError as e:
            self.tsrlibrary = None
            logger.warning('Failed creating TSRLibrary for robot "%s": %s',
                self.GetName(), e.message
            )

        self.controllers = list()
        self.manipulators = list()
        self.configurations = named_config.ConfigurationLibrary()
        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        # Standard, commonly-used OpenRAVE plugins.
        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)

    def __dir__(self):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__. 
        self = bind.InstanceDeduplicator.get_canonical(self)

        # Add planning methods to the tab-completion list.
        method_names = set(self.__dict__.keys())
        method_names.update(self.planner.get_planning_method_names())
        return list(method_names)

    def __getattr__(self, name):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__. 
        self = bind.InstanceDeduplicator.get_canonical(self)
        delegate_method = getattr(self.planner, name)

        # Resolve planner calls through the robot.planner field.
        if self.planner.has_planning_method(name):
            @functools.wraps(delegate_method)
            def wrapper_method(*args, **kw_args):
                return self._PlanWrapper(delegate_method, args, kw_args) 

            return wrapper_method

        raise AttributeError('{0:s} is missing method "{1:s}".'.format(repr(self), name))

    def CloneBindings(self, parent):
        self.planner = parent.planner
        self.controllers = list()
        self.manipulators = [ Cloned(manipulator) for manipulator in parent.manipulators ]
        self.configurations = parent.configurations
        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)

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
        from openravepy import PlannerStatus
        from prpy.exceptions import PrPyException
        from prpy.util import CopyTrajectory

        # Attempt smoothing with the Parabolic Retimer first.
        smooth_traj = CopyTrajectory(traj)
        status = openravepy.planningutils.SmoothTrajectory(
            traj, 0.99, 0.99, 'ParabolicSmoother', '')
        if status in [PlannerStatus.HasSolution,
                      PlannerStatus.InterruptedWithSolution]:
            return smooth_traj

        # If this fails, fall back on the Linear Retimer.
        # (Linear retiming should always work, but will produce a
        # slower path where the robot stops at each waypoint.)
        logger.warning(
            "SmoothTrajectory failed, using LinearTrajectoryRetimer. "
            "Robot will stop at each waypoint.")
        retimed_traj = CopyTrajectory(traj)
        status = openravepy.planningutils.RetimeTrajectory(
            traj, False, 0.99, 0.99, 'LinearTrajectoryRetimer', '')
        if status in [PlannerStatus.HasSolution,
                      PlannerStatus.InterruptedWithSolution]:
            return retimed_traj
        raise PrPyException("Path retimer failed with status '{:s}'"
                            .format(status))

    def ExecuteTrajectory(self, traj, retime=True, timeout=None, **kw_args):
        """
        Executes a trajectory and optionally waits for it to finish.
        @param traj input trajectory
        @param retime optionally retime the trajectory before executing it
        @param timeout duration to wait for execution
        @returns final executed trajectory
        """
        # Check if this is a base trajectory.
        has_base = hasattr(self, 'base')
        needs_base = util.HasAffineDOFs(traj.GetConfigurationSpecification())
        if needs_base and not has_base:
            raise ValueError('Unable to execute affine DOF trajectory; robot does'\
                             ' not have a MobileBase.')

        # TODO: Throw an error if the trajectory contains both normal DOFs and
        # affine DOFs.

        if retime:
            # Retime a manipulator trajectory.
            if not needs_base:
                traj = self.RetimeTrajectory(traj)
            # Retime a base trajectory.
            else:
                max_vel = [ self.GetAffineTranslationMaxVels()[0],
                            self.GetAffineTranslationMaxVels()[1],
                            self.GetAffineRotationAxisMaxVels()[2] ]
                max_accel = [3.*v for v in max_vel]
                openravepy.planningutils.RetimeAffineTrajectory(traj, max_vel,
                                                                max_accel, False)

        self.GetController().SetPath(traj)

        active_manipulators = self.GetTrajectoryManipulators(traj)
        active_controllers = []
        for active_manipulator in active_manipulators:
            if hasattr(active_manipulator, 'controller'):
                active_controllers.append(active_manipulator.controller)

        if needs_base:
            active_controllers.append(self.base.controller)

        util.WaitForControllers(active_controllers, timeout=timeout)
        return traj

    def ViolatesVelocityLimits(self, traj):
        """
        Checks a trajectory for velocity limit violations
        @param traj input trajectory
        """
        # Get the limits that pertain to this trajectory
        all_velocity_limits = self.GetDOFVelocityLimits()
        traj_indices = util.GetTrajectoryIndices(traj)
        velocity_limits = [all_velocity_limits[idx] for idx in traj_indices]

        # Get the velocity group from the configuration specification so
        #  that we know the offset and number of dofs
        config_spec = traj.GetConfigurationSpecification()
        num_waypoints = traj.GetNumWaypoints()
        
        # Check for the velocity group
        has_velocity_group = True
        try:
            config_spec.GetGroupFromName('joint_velocities')
        except openravepy.openrave_exception:
            logging.warn('Trajectory does not have joint velocities defined')
            has_velocity_group = False

        # Now check all the waypoints
        for idx in range(0, num_waypoints):

            wpt = traj.GetWaypoint(idx)
            
            if has_velocity_group:
                # First check the velocities defined for the waypoint
                velocities = config_spec.ExtractJointValues(wpt, self, traj_indices, 1)
                for vidx in range(len(velocities)):
                    if (velocities[vidx] > velocity_limits[vidx]):
                        logging.warn('Velocity for waypoint %d joint %d violates limits (value: %0.3f, limit: %0.3f)' % 
                                     (idx, vidx, velocities[vidx], velocity_limits[vidx]))
                        return True

            # Now check the velocities calculated by differencing positions
            dt = config_spec.ExtractDeltaTime(wpt)
            values = config_spec.ExtractJointValues(wpt, self, traj_indices, 0)

            if idx > 0:
                diff_velocities = numpy.fabs(values - prev_values)/(dt - prev_dt)
                for vidx in range(len(diff_velocities)):
                    if (diff_velocities[vidx] > velocity_limits[vidx]):
                        logging.warn('Velocity for waypoint %d joint %d violates limits (value: %0.3f, limit: %0.3f)' % 
                                     (idx, vidx, diff_velocities[vidx], velocity_limits[vidx]))
                        return True

            # Set current to previous
            prev_dt = dt
            prev_values = values

        return False

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
