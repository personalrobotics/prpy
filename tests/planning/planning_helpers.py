import numpy
from prpy.planning.base import BasePlanner, PlanningMethod

class BasePlannerTest(object):
    is_setup = False
    active_dof_indices = range(7)

    # Feasible start/goal pair.
    config_feasible_start = numpy.array([
        +2.35061574,  0.61043555,  0.85000000,  1.80684444, -0.08639935,
        -0.69750474,  1.31656172
    ])
    config_feasible_goal = numpy.array([
        -0.84085883,  1.44573701,  0.20000000,  1.72620231, -0.81124757,
        -1.39363597,  1.29233111
    ])

    # This configuration is in collision with the environment, but is not in
    # self-collision.
    config_env_collision = numpy.array([
        +3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
        +2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
        -2.07807269e-01
    ])

    # This configuration is in self-collision, but is not in collision with the
    # environment.
    config_self_collision = numpy.array([
        2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
        2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
        0.00000000e+00
    ])

    # Waypoints that can be pair-wise connected by straight line paths.
    waypoint1 = numpy.array([
        1.12376031,  0.60576977, -0.05000000,
        1.33403907,  0.44772461, -0.31481177,
        1.90265540
    ])
    waypoint2 = numpy.array([
        1.53181533,  0.80270404, -0.05000000,
        1.75341989,  0.21348846, -0.91026757,
        1.59603932
    ])
    waypoint3 = numpy.array([
        1.50376031,  0.60576977, -0.05000000,
        1.33403907,  0.44772461, -0.31481177,
        1.90265540
    ])

    def setUp(self):
        import openravepy

        if not self.is_setup:
            openravepy.RaveInitialize(True)
            openravepy.misc.InitOpenRAVELogging()
            openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)
            self.is_setup = True

        self.env = openravepy.Environment()
        with self.env:
            self.env.Load('data/wamtest2.env.xml')
            self.robot = self.env.GetRobot('BarrettWAM')
            self.manipulator = self.robot.GetManipulator('arm')

            self.env.Remove(self.env.GetKinBody('floor'))

            self.robot.SetActiveManipulator(self.manipulator)
            self.robot.SetActiveDOFs(self.active_dof_indices)

        self.planner = self.planner_factory()

        for base_cls in self.__class__.__bases__:
            if base_cls != BasePlannerTest and hasattr(base_cls, 'setUp'):
                base_cls.setUp(self)

    def tearDown(self):
        for base_cls in reversed(self.__class__.__bases__):
            if base_cls != BasePlannerTest and hasattr(base_cls, 'tearDown'):
                base_cls.tearDown(self)

        self.env.Destroy()

    def ValidatePath(self, path):
        self.assertEquals(path.GetEnv(), self.env)
        self.assertEquals(self.robot.GetActiveConfigurationSpecification('linear'),
                          path.GetConfigurationSpecification())
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)

    def CollisionCheckPath(self, traj):
        from openravepy import Interval, Planner

        # NOTE: This assumes that the trajectory only contains joint_values.
        OpenStart = Interval.OpenStart

        params = Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)

        # Check the first waypoint for collision. We do this outside of the
        # loop so we can run all collision checks with OpenEnd. This prevents
        # us from collision checking the intermediate waypoints twice.
        prev_waypoint = traj.GetWaypoint(0)
        check = params.CheckPathAllConstraints(prev_waypoint, prev_waypoint,
                                               [], [], 0., OpenStart)
        if check != 0:
            return True

        # Check the remainder of the path.
        for iwaypoint in xrange(1, traj.GetNumWaypoints() - 1):
            curr_waypoint = traj.GetWaypoint(iwaypoint)

            check = params.CheckPathAllConstraints(
                prev_waypoint, curr_waypoint, [], [], 0., OpenStart)
            if check != 0:
                return True

            prev_waypoint = curr_waypoint

        return False

    def ComputeArcLength(self, traj):
        distance = 0.

        for iwaypoint in xrange(1, traj.GetNumWaypoints()):
            prev_waypoint = traj.GetWaypoint(iwaypoint - 1)
            curr_waypoint = traj.GetWaypoint(iwaypoint)
            distance += numpy.linalg.norm(curr_waypoint - prev_waypoint)

        return distance

    def assertTransformClose(self, actual_pose, expected_pose,
                             linear_tol=1e-3, angular_tol=1e-3):
        rel_pose = numpy.dot(numpy.linalg.inv(actual_pose), expected_pose)
        distance = numpy.linalg.norm(rel_pose[0:3, 3])

        # This value should be in the range [ -1, 1 ], but it may be slightly
        # outside due to numerical imprecision. This can cause arccos to
        # return NaN, so we explicitly project it first.
        angle_cos = (numpy.trace(rel_pose[0:3, 0:3]) - 1.) / 2.
        angle_cos = min(max(angle_cos, -1.), 1.)
        angle = numpy.arccos(angle_cos)

        self.assertLessEqual(distance, linear_tol)
        self.assertLessEqual(angle, angular_tol)


class MockPlanner(BasePlanner):
    def __init__(self, delay=False):
        from threading import Condition

        BasePlanner.__init__(self)

        self.num_calls = 0
        self.delay = delay
        self.start_condition = Condition()
        self.done_condition = Condition()
        self.running = False

    def wait_for_start(self):
        if self.delay:
            with self.start_condition:
                while not self.running:
                    self.start_condition.wait()

    def finish(self):
        with self.done_condition:
            self.done_condition.notify_all()

    def _PlanGeneric(self, delegate_method, *args, **kw_args):
        # Notify other threads that the planner has started.
        with self.start_condition:
            self.num_calls += 1
            self.running = True
            self.start_condition.notify_all()

        # Optionally wait before returning
        if self.delay:
            with self.done_condition:
                self.done_condition.wait()

        # Compute the return value and return.
        try:
            return delegate_method(*args, **kw_args)
        finally:
            with self.start_condition:
                self.running = False


class SuccessPlanner(MockPlanner):
    def __init__(self, template_traj, delay=False):
        from openravepy import RaveCreateTrajectory

        MockPlanner.__init__(self, delay=delay)
        
        with self.env:
            # Clone the template trajectory into the planning environment.
            self.traj = RaveCreateTrajectory(self.env, template_traj.GetXMLId())
            self.traj.Clone(template_traj, 0)

    @PlanningMethod
    def PlanTest(self, robot, defer=False):
        assert not defer

        def Success_impl(robot):
            import numpy
            from openravepy import RaveCreateTrajectory

            cspec = robot.GetActiveConfigurationSpecification()
            traj = RaveCreateTrajectory(self.env, 'GenericTrajectory')
            traj.Init(cspec)
            traj.Insert(0, numpy.zeros(cspec.GetDOF()))
            traj.Insert(1, numpy.ones(cspec.GetDOF()))
            return traj

        return self._PlanGeneric(Success_impl, robot)


class FailPlanner(MockPlanner):
    @PlanningMethod
    def PlanTest(self, robot, defer=False):
        assert not defer

        def Failure_impl(robot):
            from prpy.planning import PlanningError

            raise PlanningError('FailPlanner')

        return self._PlanGeneric(Failure_impl, robot)


class MetaPlannerTests(object):
    def setUp(self):
        from openravepy import Environment, RaveCreateTrajectory

        self.join_timeout = 5.0
        self.env = Environment()
        self.env.Load('data/lab1.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')

        # Create a valid trajectory used to test planner successes.
        cspec = self.robot.GetActiveConfigurationSpecification()
        self.traj = RaveCreateTrajectory(self.env, 'GenericTrajectory')
        self.traj.Init(cspec)
        self.traj.Insert(0, numpy.zeros(cspec.GetDOF()))
        self.traj.Insert(1, numpy.ones(cspec.GetDOF()))

