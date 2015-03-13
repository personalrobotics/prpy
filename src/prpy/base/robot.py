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
import prpy.util
from ..clone import Clone, Cloned
from ..tsr.tsrlibrary import TSRLibrary
from ..planning.base import Sequence 
from ..planning.ompl import OMPLSimplifier
from ..planning.retimer import ParabolicRetimer, ParabolicSmoother
from ..planning.mac_smoother import MacSmoother

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

        # Path post-processing for execution. This includes simplification of
        # the geometric path, retiming a path into a trajectory, and smoothing
        # (joint simplificaiton and retiming).
        self.simplifier = OMPLSimplifier()
        self.retimer = ParabolicRetimer()
        self.smoother = Sequence(
            ParabolicSmoother(),
            self.retimer
        )

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

        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)
        self.ompl_simplifier = OMPLSimplifier()


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

    def ExecutePath(self, path, simplify=True, smooth=True, defer=False,
                    timeout=1., **kwargs):

        logger.debug('Begin ExecutePath')

        def do_execute(path, simplify, smooth, timeout, **kwargs):

            with Clone(self.GetEnv()) as cloned_env:
                traj_dofs = util.GetTrajectoryIndices(path)
                cloned_env.Cloned(self).SetActiveDOFs(traj_dofs)
                cloned_robot = cloned_env.Cloned(self)

                if simplify:
                    path = self.simplifier.ShortcutPath(cloned_robot, path, defer=False,
                                                        timeout=timeout, **kwargs)

                retimer = self.smoother if smooth else self.retimer
                cloned_timed_traj = retimer.RetimeTrajectory(cloned_robot, path, defer=False, **kwargs)

                # Copy the trajectory back to the original environment.
                from ..util import CopyTrajectory
                timed_traj = CopyTrajectory(cloned_timed_traj, env=self.GetEnv())

            return self.ExecuteTrajectory(timed_traj, defer=False, **kwargs)

        if defer:
            from trollius.executor import get_default_executor
            from trollius.futures import wrap_future

            executor = kwargs.get('executor') or get_default_executor()
            return wrap_future(
                executor.submit(do_execute,
                    path, simplify=simplify, smooth=smooth, timeout=timeout,
                    **kwargs
                )
            )
        else:
            return do_execute(path, simplify=simplify, smooth=smooth,
                              timeout=timeout, **kwargs)

    def ExecuteTrajectory(self, traj, defer=False, timeout=None, period=0.01, **kw_args):
        # TODO: Verify that the trajectory is timed.
        # TODO: Check if this trajectory contains the base.

        logger.debug('Begin ExecuteTrajectory')
        needs_base = util.HasAffineDOFs(traj.GetConfigurationSpecification())

        self.GetController().SetPath(traj)

        active_manipulators = self.GetTrajectoryManipulators(traj)
        active_controllers = [
            active_manipulator.controller \
            for active_manipulator in active_manipulators \
            if hasattr(active_manipulator, 'controller')
        ]

        if needs_base:
            if hasattr(self, 'base') and hasattr(self.base, 'controller'):
                active_controllers.append(self.base.controller)

        if defer:
            import time
            import trollius

            @trollius.coroutine
            def do_poll():
                time_stop = time.time() + (timeout if timeout else numpy.inf)

                while time.time() <= time_stop:
                    is_done = all(controller.IsDone()
                                  for controller in active_controllers)
                    if is_done:
                        raise trollius.Return(traj)

                    yield trollius.From(trollius.sleep(period))

                raise trollius.Return(None)

            return trollius.async(do_poll())
        else:
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
        config_spec = self.GetActiveConfigurationSpecification('linear')

        # Call the planner.
        result = planning_method(self, *args, **kw_args)

        def postprocess_trajectory(traj):
            # Strip inactive DOFs from the trajectory.
            openravepy.planningutils.ConvertTrajectorySpecification(
                traj, config_spec
            )

        # Return either the trajectory result or a future to the result.
        if kw_args.get('defer', False):
            import trollius

            # Perform postprocessing on a future trajectory.
            @trollius.coroutine
            def defer_trajectory(traj_future, kw_args):
                # Wait for the planner to complete.
                traj = yield trollius.From(traj_future)

                postprocess_trajectory(traj)

                # Optionally execute the trajectory.
                if kw_args.get('execute', True):
                    # We know defer = True if we're in this function, so we
                    # don't have to set it explicitly.
                    traj = yield trollius.From(
                        self.ExecutePath(traj, **kw_args)
                    )

                raise trollius.Return(traj)

            return trollius.Task(defer_trajectory(result, kw_args))
        else:
            postprocess_trajectory(result)

            # Optionally execute the trajectory.
            if kw_args.get('execute', True):
                result = self.ExecutePath(result, **kw_args)

            return result
