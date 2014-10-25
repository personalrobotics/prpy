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

import logging, openravepy, numpy
from .. import util
from robot import Robot
import prpy.rave
from .. import exceptions

class WAMRobot(Robot):
    def __init__(self):
        Robot.__init__(self)

        # Optional MacTrajectory retimer. If this fails, we'll fall back on
        # executing generic trajectories.
        self.mac_retimer = openravepy.RaveCreatePlanner(self.GetEnv(), 'MacRetimer')

        # Trajectory blending.
        self.trajectory_module = prpy.rave.load_module(self.GetEnv(), 'Trajectory', self.GetName())
        import manipulation2.trajectory
        manipulation2.trajectory.bind(self.trajectory_module)

    def CloneBindings(self, parent):
        Robot.CloneBindings(self, parent)

        self.mac_retimer = None
        self.trajectory_module = prpy.rave.load_module(self.GetEnv(), 'Trajectory', self.GetName())
        #import manipulation2.trajectory
        #manipulation2.trajectory.bind(self.trajectory_module)

    def RetimeTrajectory(self, traj, max_jerk=30.0, synchronize=False,
                         stop_on_stall=True, stop_on_ft=False, force_direction=None,
                         force_magnitude=None, torque=None, **kw_args):
        """Retime a generic OpenRAVE trajectory into a timed for OWD.
        First, try to retime the trajectory into a MacTrajectory using OWD's
        MacRetimer. If MacRetimer is not available, then fall back on the
        default OpenRAVE retimer.
        @param traj input trajectory
        @param max_jerk maximum jerk allowed during retiming
        @param synchronize enable synchronization between multiple OWD processes
        @param stop_on_stall cancel the trajectory if stall torques are exceeded
        @param stop_on_ft cancel the trajectory on force/torque input
        @param force_direction direction vector used for stop_on_ft
        @param force_magnitude force threshold for stop_on_ft
        @param torque torque threshold for stop_on_ft
        @return timed trajectory
        """
        # Fall back on the standard OpenRAVE retimer if MacTrajectory is not
        # available.
        if self.mac_retimer is None:
            return Robot.RetimeTrajectory(self, traj, **kw_args)

        # check the number of 
        num_waypoints = traj.GetNumWaypoints()
        if num_waypoints < 2:
            logging.warn('RetimeTrajectory received trajectory with less than 2 waypoints. Skipping retime.')
            return traj

        # Create a MacTrajectory with timestamps, joint values, velocities,
        # accelerations, and blend radii.
        generic_config_spec = traj.GetConfigurationSpecification()
        generic_angle_group = generic_config_spec.GetGroupFromName('joint_values')
        path_config_spec = openravepy.ConfigurationSpecification()
        path_config_spec.AddDeltaTimeGroup()
        path_config_spec.AddGroup(generic_angle_group.name, generic_angle_group.dof, '')
        path_config_spec.AddDerivativeGroups(1, False);
        path_config_spec.AddDerivativeGroups(2, False);
        path_config_spec.AddGroup('owd_blend_radius', 1, 'next')
        path_config_spec.ResetGroupOffsets()

        # Initialize the MacTrajectory.
        mac_traj = openravepy.RaveCreateTrajectory(self.GetEnv(), 'MacTrajectory')
        mac_traj.Init(path_config_spec)

        # Copy the joint values and blend radii into the MacTrajectory.
        for i in xrange(num_waypoints):
            waypoint = traj.GetWaypoint(i, path_config_spec)
            mac_traj.Insert(i, waypoint, path_config_spec)

        # Serialize the planner parameters.
        params = [ 'max_jerk', str(max_jerk) ]
        if stop_on_stall:
            params += [ 'cancel_on_stall' ]
        if stop_on_ft:
            force_threshold = force_magnitude * numpy.array(force_direction)
            params += [ 'cancel_on_ft' ]
            params += [ 'force_threshold' ] + map(str, force_threshold)
            params += [ 'torque_threshold' ] + map(str, torque)
        if synchronize:
            params += [ 'synchronize' ]

        # Retime the MacTrajectory with the MacRetimer.
        params_str = ' '.join(params)
        retimer_params = openravepy.Planner.PlannerParameters()
        retimer_params.SetExtraParameters(params_str)

        with self:
            self.mac_retimer.InitPlan(self, retimer_params)
            self.mac_retimer.PlanPath(mac_traj)
            return mac_traj

    def BlendTrajectory(self, traj, maxsmoothiter=None, resolution=None,
                        blend_radius=0.2, blend_attempts=4, blend_step_size=0.05,
                        linearity_threshold=0.1, ignore_collisions=None, **kw_args):
        """Blend a trajectory for execution in OWD.
        Blending a trajectory allows the MacRetimer to smoothly accelerate
        through waypoints. If a blend radius is not specified, it defaults to
        zero and the controller must come to a stop at each waypoint. This adds
        the \tt blend_radius group to the input trajectory.
        @param traj input trajectory
        @return blended trajectory
        """
        with self:
            return self.trajectory_module.blendtrajectory(traj=traj, execute=False,
                    maxsmoothiter=maxsmoothiter, resolution=resolution,
                    blend_radius=blend_radius, blend_attempts=blend_attempts,
                    blend_step_size=blend_step_size, linearity_threshold=linearity_threshold,
                    ignore_collisions=ignore_collisions)

    def ExecuteTrajectory(self, traj, timeout=None, blend=True, retime=True, limit_tolerance=1e-3, synchronize=True, **kw_args):
        """Execute a trajectory.
        By default, this function retimes, blends, and adds the stop_on_stall
        flag to all trajectories. This behavior can be overriden using the \tt
        blend and \tt retime flags or by passing the appropriate \tt **kw_args
        arguments to the blender and retimer. By default, this function blocks
        until trajectory execution finishes. This can be changed by changing
        the timeout parameter to a maximum number of seconds. Pass a timeout of
        zero to return instantly.
        @param traj trajectory to execute
        @param timeout blocking execution timeout
        @param blend flag for computing blend radii before execution
        @param retime flag for retiming the trajectory before execution
        @return executed_traj including blending and retiming
        """
        # Query the active manipulators based on which DOF indices are
        # included in the trajectory.
        active_manipulators = self.GetTrajectoryManipulators(traj)
        needs_synchronization = synchronize and len(active_manipulators) > 1
        sim_flags = [ manipulator.simulated for manipulator in active_manipulators ]

        if needs_synchronization and any(sim_flags) and not all(sim_flags):
            raise exceptions.SynchronizationException(
                'Unable to execute synchronized trajectory with'
                ' a mixture of simulated and real controllers.')

        # Disallow trajectories that include both the base and the arms when
        # not in simulation mode. We can't guarantee synchronization in this case.
        def has_affine_dofs(traj):
            def has_group(cspec, group_name):
                try:
                    cspec.GetGroupFromName(group_name)
                    return True
                except openravepy.openrave_exception:
                    return False

            cspec = traj.GetConfigurationSpecification()
            return (has_group(cspec, 'affine_transform')
                 or has_group(cspec, 'affine_velocities')
                 or has_group(cspec, 'affine_accelerations'))

        needs_arms = bool(active_manipulators)
        needs_base = has_affine_dofs(traj)
        arms_simulated = all(sim_flags)
        
        if needs_base and needs_arms and not arms_simulated and not self.base.simulated:
            raise NotImplementedError('Joint arm-base trajectories are not supported on hardware.')

        # Optionally blend and retime the trajectory before execution. Retiming
        # creates a MacTrajectory that can be directly executed by OWD.
        if needs_arms:
            active_indices = util.GetTrajectoryIndices(traj)
            with self:
                self.SetActiveDOFs(active_indices)
                if blend:
                    traj = self.BlendTrajectory(traj)

                # Check if the first point is not in limits.
                unclamped_dof_values = self.GetActiveDOFValues()

                first_waypoint = traj.GetWaypoint(0)
                cspec = traj.GetConfigurationSpecification()
                first_dof_values = cspec.ExtractJointValues(first_waypoint, self, active_indices, 0)
                lower_limits, upper_limits = self.GetActiveDOFLimits()

                for i in xrange(len(first_dof_values)):
                    if numpy.allclose(first_dof_values[i], lower_limits[i], atol=limit_tolerance) and unclamped_dof_values[i] < lower_limits[i]:
                        first_dof_values[i] = unclamped_dof_values[i]
                        logging.warn('Unclamped DOF %d from lower limit.', active_indices[i])
                    elif numpy.allclose(first_dof_values[i], upper_limits[i], atol=limit_tolerance) and unclamped_dof_values[i] > upper_limits[i]:
                        first_dof_values[i] = unclamped_dof_values[i]
                        logging.warn('Unclamped DOF %d from upper limit.', active_indices[i])

                # Snap the first point to the current configuration.
                cspec.InsertJointValues(first_waypoint, first_dof_values, self, active_indices, 0)
                traj.Insert(0, first_waypoint, True)

                # This must be last because MacTrajectories are immutable.
                # TODO: This may break if the retimer clamps DOF values to the joint limits.
                if retime:
                    traj = self.RetimeTrajectory(traj, synchronize=needs_synchronization, **kw_args)

        if needs_base:
            # Retime the base trajectory in simulation.
            if retime:# and self.base.simulated:
                max_vel = numpy.concatenate((self.GetAffineTranslationMaxVels()[:2],
                                             [ self.GetAffineRotationQuatMaxVels() ]))
                max_accel = 3 * max_vel
                openravepy.planningutils.RetimeAffineTrajectory(traj, max_vel,
                                                                max_accel, False)

        # Can't execute trajectories with less than two waypoints
        if traj.GetNumWaypoints() < 2:
            logging.warn('Unable to execute trajectories with less than 2 waypoints. Skipping execution.')
            return traj

        # Synchronization implicitly executes on all manipulators.
        if needs_synchronization:
            running_manipulators = set(self.manipulators)
        else:
            running_manipulators = set(active_manipulators)

        # Reset old trajectory execution flags
        for manipulator in active_manipulators:
            manipulator.ClearTrajectoryStatus()

        # FIXME: The IdealController attempts to sample the MacTrajectory in simulation.
        # self.GetController().SetPath(traj)

        # Wait for trajectory execution to finish.
        running_controllers = [ manipulator.controller for manipulator in running_manipulators ]
        if needs_base:
            running_controllers += [ self.base.controller ]

        for controller in running_controllers:
            controller.SetPath(traj)

        is_done = util.WaitForControllers(running_controllers, timeout=timeout)
            
        # Request the controller status from each manipulator.
        if is_done:
            with self.GetEnv():
                for manipulator in active_manipulators:
                    status = manipulator.GetTrajectoryStatus()
                    if status == 'aborted':
                        raise exceptions.TrajectoryAborted('Trajectory aborted for %s' % manipulator.GetName())
                    elif status == 'stalled':
                        raise exceptions.TrajectoryStalled('Trajectory stalled for %s' % manipulator.GetName())

                    if manipulator.simulated:
                        manipulator.controller.Reset()

        return traj
