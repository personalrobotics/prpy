import numpy, openravepy

class RenderTrajectory:
    """
    Context manager for rendering trajectories in the OpenRAVE viewer. This
    renders a trajectory as a smooth curve. The curve is are removed when the exit
    handler is called.
    @param robot robot executing the trajectory
    @param traj input trajectory
    @param num_samples number of samples to use for interpolation
    @param radius sphere radius for waypoint indicators
    @param color interpolated line color
    @param interpolation flag to enable or disable path rendering
    @param waypoints flag to enable or disable waypoint rendering
    """
    def __init__(self, robot, traj, num_samples=20, linewidth=2, color=[1, 0, 0, 1]):
        self.env = robot.GetEnv()
        self.robot = robot
        self.handles = list()

        # Rendering options.
        self.num_samples = num_samples
        self.linewidth = linewidth
        self.color = numpy.array(color, dtype='float')

        # Clone and retime the trajectory.
        self.traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')
        self.traj.Clone(traj, 0)
        openravepy.planningutils.RetimeTrajectory(self.traj)

    def __enter__(self):
        
        with self.env:
            with self.robot.CreateRobotStateSaver():
                config_spec = self.traj.GetConfigurationSpecification()
                manipulators = self.robot.GetTrajectoryManipulators(self.traj)

                for manipulator in manipulators:
                    arm_indices = manipulator.GetArmIndices()

                    # Skip manipulators that don't have render_offset set.
                    if hasattr(manipulator, "render_offset"):
                        render_offset = manipulator.render_offset
                        if render_offset is None:
                            continue
                    else:
                        render_offset = [0., 0., 0., 1.]

                    # Evenly interpolate joint values throughout the entire trajectory.
                    interpolated_points = list()
                    for t in numpy.linspace(0, self.traj.GetDuration(), self.num_samples):
                        waypoint = self.traj.Sample(t)
                        joint_values = config_spec.ExtractJointValues(waypoint, self.robot, arm_indices)
                        manipulator.SetDOFValues(joint_values)
                        hand_pose = manipulator.GetEndEffectorTransform()
                        render_position = numpy.dot(hand_pose, render_offset)
                        interpolated_points.append(render_position[0:3])

                    # Render a line through the interpolated points.
                    interpolated_points = numpy.array(interpolated_points)
                    if len(interpolated_points) > 0:
                        handle = self.env.drawlinestrip(interpolated_points, self.linewidth, self.color)
                        handle.SetShow(True)
                        self.handles.append(handle)

    def __exit__(self, *exc_info):
        with self.env:
            del self.handles


