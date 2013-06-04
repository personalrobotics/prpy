import contextlib, logging, numpy, openravepy, rospkg
from base import Planner, PlanningError, UnsupportedPlanningError

class CHOMPPlanner(Planner):
    Planner.register_type('PlanToConfiguration')
    Planner.register_type('PlanToEndEffectorPose')

    def __init__(self):
        self.planner_env = openravepy.Environment()
        try:
            from orcdchomp import orcdchomp
            self.module = openravepy.RaveCreateModule(self.planner_env, 'orcdchomp')
        except ImportError:
            raise UnsupportedPlanningError('Unable to import orcdchomp.')
        except openravepy.openrave_exception as e:
            raise UnsupportedPlanningError('Unable to create orcdchomp module: %s' % e)

        if self.module is None:
            raise UnsupportedPlanningError('Failed loading CBiRRT module.')

        self.initialized = False
        orcdchomp.bind(self.module)

    def GetName(self):
        return 'CHOMP'

    def PlanToConfiguration(self, robot, goal, lambda_=100.0, n_iter=100, **kw_args):
        if not self.initialized:
            raise UnsupportedPlanningError('CHOMP requires a distance field.')

        try:
            return self.module.runchomp(robot=robot, adofgoal=goal,
                                        lambda_=lambda_, n_iter=n_iter, **kw_args)
        except RuntimeError, e:
            raise PlanningError(str(e))

    def PlanToEndEffectorPose(self, robot, goal_pose, lambda_=100.0, n_iter=100, goal_tolerance=0.01, **kw_args):
        # CHOMP only supports start sets. Instead, we plan backwards from the
        # goal TSR to the starting configuration. Afterwards, we reverse the
        # trajectory.
        # TODO: Replace this with a proper goalset CHOMP implementation.
        manipulator_index = robot.GetActiveManipulatorIndex()
        goal_tsr = prrave.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        start_config = robot.GetActiveDOFValues()

        if not self.initialized:
            raise UnsupportedPlanningError('CHOMP requires a distance field.')

        try:
            traj = self.module.runchomp(robot=robot, adofgoal=start_config, start_tsr=goal_tsr)
            traj = openravepy.planningutils.ReverseTrajectory(traj)
        except RuntimeError, e:
            raise PlanningError(str(e))

        # Verify that CHOMP didn't converge to the wrong goal. This is a
        # workaround for a bug in GSCHOMP where the constraint projection
        # fails because of joint limits.
        config_spec = traj.GetConfigurationSpecification()
        last_waypoint = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
        final_config = config_spec.ExtractJointValues(last_waypoint, robot, robot.GetActiveDOFIndices())
        robot.SetActiveDOFValues(final_config)
        final_pose = robot.GetActiveManipulator().GetEndEffectorTransform()

        # TODO: Also check the orientation.
        goal_distance = numpy.linalg.norm(final_pose[0:3, 3] - goal_pose[0:3, 3])
        if goal_distance > goal_tolerance:
            raise planner.PlanningError('CHOMP deviated from the goal pose by {0:f} meters.'.format(goal_distance))

        return traj

    def ComputeDistanceField(self, robot):
        env = robot.GetEnv()
        with env:
            # Save the state so we can restore it later.
            savers = [ body.CreateKinBodyStateSaver() for body in env.GetBodies() ]

            with contextlib.nested(*savers):
                # Disable everything.
                for body in env.GetBodies():
                    body.Enable(False)

                # Compute the distance field for the non-spherized parts of HERB. This
                # includes everything that isn't attached to an arm. Otherwise the
                # initial arm will be incorrectly added to the distance field.
                robot.Enable(True)
                logging.info("Creating the robot's distance field.")
                proximal_joints = [ manip.GetArmIndices()[0] for manip in robot.GetManipulators() ]
                for link in robot.GetLinks():
                    for proximal_joint in proximal_joints:
                        if robot.DoesAffect(proximal_joint, link.GetIndex()):
                            link.Enable(False)

                cache_path = self.GetCachePath(robot)
                self.module.computedistancefield(robot, cache_filename=cache_path)
                robot.Enable(False)

                # Compute separate distance fields for all other objects.
                for body in env.GetBodies():
                    if body != robot:
                        logging.info("Creating distance field for '{0:s}'.".format(body.GetName()))
                        body.Enable(True)
                        cache_path = self.GetCachePath(body)
                        self.module.computedistancefield(body, cache_filename=cache_path)
                        body.Enable(False)

        self.initialized = True 
        for saver in savers:
            saver.Restore()
            saver.Release()

    def GetCachePath(self, body):
        import os
        cache_dir = rospkg.get_ros_home()
        cache_name = '{0:s}.chomp'.format(body.GetKinematicsGeometryHash())
        return os.path.join(cache_dir, cache_name)
