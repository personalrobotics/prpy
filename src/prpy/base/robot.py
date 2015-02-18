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
        self.robot_name = robot_name

        try:
            self.tsrlibrary = TSRLibrary(self, robot_name=robot_name)
        except ValueError as e:
            self.tsrlibrary = None
            logger.warning('Failed creating TSRLibrary for robot "%s": %s',
                           self.GetName(), e.message)

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
        Robot.__init__(self, parent.robot_name)
        self.planner = parent.planner
        self.manipulators = [Cloned(manipulator, into=self.GetEnv())
                             for manipulator in parent.manipulators]
        self.configurations = parent.configurations

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
        """Compute timing information for a trajectory.

        This function computes timing information for a trajectory, populating
        the trajectory's deltatime group. Timing information is necessary for
        successful execution in simulation.

        Note that this method does NOT modify the trajectory in-place, but
        instead returns a timed version of the trajectory.

        @param traj input trajectory
        @return timed output trajectory
        """
        from openravepy import PlannerStatus
        from prpy.exceptions import PrPyException
        from prpy.util import CopyTrajectory, SimplifyTrajectory

        # Simplify the trajectory before performing retiming.
        if traj.GetDuration() == 0.0:
            traj = SimplifyTrajectory(traj, self)

        # Attempt smoothing with the Parabolic Retimer first.
        smooth_traj = CopyTrajectory(traj)
        status = openravepy.planningutils.SmoothTrajectory(
            smooth_traj, 0.99, 0.99, 'ParabolicSmoother', '')
        if status in [PlannerStatus.HasSolution,
                      PlannerStatus.InterruptedWithSolution]:
            return smooth_traj

        # If this fails, fall back on the Linear Retimer.
        # (Linear retiming should always work, but will produce a
        # slower path where the robot stops at each waypoint.)
        logger.warning(
            "SmoothTrajectory failed, using ParabolicTrajectoryRetimer. "
            "Robot will stop at each waypoint: {:d}"
            .format(traj.GetNumWaypoints()))
        retimed_traj = CopyTrajectory(traj)
        status = openravepy.planningutils.RetimeTrajectory(
            retimed_traj, False, 1., 1., 'ParabolicTrajectoryRetimer', '')
        if status in [PlannerStatus.HasSolution,
                      PlannerStatus.InterruptedWithSolution]:
            return retimed_traj
        raise PrPyException("Path retimer failed with status '{:s}'"
                            .format(status))

    def ExecuteTrajectory(self, traj, retime=True, timeout=None,
                          defer=False, executor=None, **kw_args):
        """
        Executes a trajectory and optionally waits for it to finish.

        Passing `defer=True` to this function submits it for background
        execution, and returns a Future which contains the result.

        @param traj input trajectory
        @param retime optionally retime the trajectory before executing it
        @param timeout duration to wait for execution
        @param defer return a future to this function and run in the background
        @returns final executed trajectory or a Future to this result
        """
        # If the planning call is deferred, submit the call to the executor.
        if defer is True:
            from trollius.executor import get_default_executor
            from trollius.futures import wrap_future
            executor = executor or get_default_executor()
            return wrap_future(executor.submit(self.ExecuteTrajectory,
                                               traj, retime, timeout,
                                               defer=False, **kw_args))

        # Check if this is a base trajectory.
        has_base = hasattr(self, 'base')
        needs_base = util.HasAffineDOFs(traj.GetConfigurationSpecification())
        if needs_base and not has_base:
            raise ValueError('Unable to execute affine DOF trajectory; '
                             'robot does not have a MobileBase.')

        # TODO: Throw an error if the trajectory contains both normal DOFs and
        # affine DOFs.

        if retime:
            # Retime a manipulator trajectory.
            if not needs_base:
                traj = self.RetimeTrajectory(traj)
            # Retime a base trajectory.
            else:
                max_vel = [self.GetAffineTranslationMaxVels()[0],
                           self.GetAffineTranslationMaxVels()[1],
                           self.GetAffineRotationAxisMaxVels()[2]]
                max_accel = [3.*v for v in max_vel]
                openravepy.planningutils.RetimeAffineTrajectory(
                    traj, max_vel, max_accel, False)

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
        config_spec = self.GetActiveConfigurationSpecification()
        result = planning_method(self, *args, **kw_args)

        # Define the post processing steps for the trajectory.
        def postprocess_trajectory(traj, kw_args):

            # Strip inactive DOFs from the trajectory.
            openravepy.planningutils.ConvertTrajectorySpecification(
                traj, config_spec
            )

            # Optionally execute the trajectory.
            if 'execute' not in kw_args or kw_args['execute']:
                kw_args['defer'] = False
                return self.ExecuteTrajectory(traj, **kw_args)
            else:
                return traj

        # Return either the trajectory result or a future to the result.
        if 'defer' in kw_args and kw_args['defer'] is True:
            import trollius

            # Perform postprocessing on a future trajectory.
            @trollius.coroutine
            def defer_trajectory(traj_future, kw_args):
                # Wait for the planner to complete.
                traj = yield trollius.From(traj_future)

                # Submit a new task to postprocess the trajectory.
                from trollius.executor import get_default_executor
                executor = kw_args.get('executor') or get_default_executor()
                f = executor.submit(postprocess_trajectory, traj, kw_args)

                # Wait for the postprocessed trajectory to be completed.
                from trollius.futures import wrap_future
                processed_traj = yield trollius.From(wrap_future(f))
                raise trollius.Return(processed_traj)

            return trollius.Task(defer_trajectory(result, kw_args))
        else:
            return postprocess_trajectory(result, kw_args)
