import openravepy
from .. import util
from robot import Robot
import manipulation2.trajectory, prrave.rave

class WAMRobot(Robot):
    def __init__(self):
        Robot.__init__(self)

        # Optional MacTrajectory retimer. If this fails, we'll fall back on
        # executing generic trajectories.
        self.mac_retimer = openravepy.RaveCreatePlanner(self.GetEnv(), 'MacRetimer')

        # Trajectory blending.
        self.trajectory_module = prrave.rave.load_module(self.GetEnv(), 'Trajectory', self.GetName())
        manipulation2.trajectory.bind(self.trajectory_module)

    def CloneBindings(self, parent):
        Robot.CloneBindings(self, parent)

        self.mac_retimer = None
        self.trajectory_module = prrave.rave.load_module(self.GetEnv(), 'Trajectory', self.GetName())
        manipulation2.trajectory.bind(self.trajectory_module)

    def RetimeTrajectory(self, traj, max_jerk=30.0, synchronize=False,
                         stop_on_stall=True, stop_on_ft=False, force_direction=None,
                         force_magnitude=None, torque=None, **kw_args):
        """
        Retime a generic OpenRAVE trajectory into a timed MacTrajectory. If the
        MacRetimer is not available, fall back on the OpenRAVE retimer.
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
        num_waypoints = traj.GetNumWaypoints()
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
        """
        Blend a trajectory for execution in OWD. This adds the blend_radius
        group to an existing trajectory.
        @param traj input trajectory
        @return blended trajectory
        """
        with self:
            return self.trajectory_module.blendtrajectory(traj=traj, execute=False,
                    maxsmoothiter=maxsmoothiter, resolution=resolution,
                    blend_radius=blend_radius, blend_attempts=blend_attempts,
                    blend_step_size=blend_step_size, linearity_threshold=linearity_threshold,
                    ignore_collisions=ignore_collisions)

    def ExecuteTrajectory(self, traj, timeout=None, blend=True, retime=True, **kw_args):
        """
        Execute a trajectory. By default, this retimes, blends, and adds the
        stop_on_stall flag to all trajectories. Additionally, this function blocks
        until trajectory execution finishes. This can be changed by changing the
        timeout parameter to a maximum number of seconds. Pass a timeout of zero to
        return instantly.
        @param traj trajectory to execute
        @param timeout blocking execution timeout
        @param blend compute blend radii before execution
        @param retime retime the trajectory before execution
        @return executed_traj
        """
        # Query the active manipulators based on which DOF indices are
        # included in the trajectory.
        active_manipulators = self.GetTrajectoryManipulators(traj)
        needs_synchronization = len(active_manipulators) > 1
        sim_flags = [ manipulator.simulated for manipulator in active_manipulators ]

        if needs_synchronization and any(sim_flags) and not all(sim_flags):
            raise exceptions.SynchronizationException('Unable to execute synchronized trajectory with'
                                                      ' a mixture of simulated and real controllers.')

        # Optionally blend and retime the trajectory before execution. Retiming
        # creates a MacTrajectory that can be directly executed by OWD.
        active_indices = util.GetTrajectoryIndices(traj)
        with self:
            self.SetActiveDOFs(active_indices)
            if blend:
                traj = self.BlendTrajectory(traj)
            if retime:
                traj = self.RetimeTrajectory(traj, synchronize=needs_synchronization, **kw_args)

        # Synchronization implicitly executes on all manipulators.
        if needs_synchronization:
            running_manipulators = set(self.manipulators)
        else:
            running_manipulators = set(active_manipulators)

        # Reset old trajectory execution flags
        for manipulator in active_manipulators:
            manipulator.ClearTrajectoryStatus()

        self.GetController().SetPath(traj)

        # Wait for trajectory execution to finish.
        running_controllers = [ manipulator.controller for manipulator in running_manipulators ]
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

