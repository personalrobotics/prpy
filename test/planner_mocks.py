import prpy.planning, threading, openravepy, numpy

class MockPlanner(prpy.planning.BasePlanner):
    def __init__(self, delay=False):
        prpy.planning.BasePlanner.__init__(self)
        self.num_calls = 0
        self.delay = delay
        self.start_condition = threading.Condition()
        self.done_condition = threading.Condition()
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
        MockPlanner.__init__(self, delay=delay)
        
        with self.env:
            # Clone the template trajectory into the planning environment.
            self.traj = openravepy.RaveCreateTrajectory(self.env, template_traj.GetXMLId())
            self.traj.Clone(template_traj, 0)

    @prpy.planning.PlanningMethod
    def PlanTest(self, robot):
        def Success_impl(robot):
            cspec = robot.GetActiveConfigurationSpecification()
            traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')
            traj.Init(cspec)
            traj.Insert(0, numpy.zeros(cspec.GetDOF()))
            traj.Insert(1, numpy.ones(cspec.GetDOF()))
            return traj

        return self._PlanGeneric(Success_impl, robot)

class FailPlanner(MockPlanner):
    @prpy.planning.PlanningMethod
    def PlanTest(self, robot):
        def Failure_impl(robot):
            raise prpy.planning.PlanningError('FailPlanner')

        return self._PlanGeneric(Failure_impl, robot)
