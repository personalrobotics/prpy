import logging, functools, openravepy

logger = logging.getLogger('planning')

class PlanningError(Exception):
    pass

class UnsupportedPlanningError(PlanningError):
    pass

class MetaPlanningError(PlanningError):
    def __init__(self, message, errors):
        PlanningError.__init__(self, message)
        self.errors = errors

    # TODO: Print the inner exceptions.

class PlanningMethod(object):
    def __init__(self, func):
        self.func = func
        Planner.register_type(func.__name__)
        
    def __call__(self, instance, live_robot, *args, **kw_args):
        live_env = live_robot.GetEnv()
        planning_env = instance.env

        # Clone the live environment for planning.
        from openravepy import CloningOptions
        planning_env.Clone(live_env, CloningOptions.Bodies)
        planning_robot = planning_env.GetRobot(live_robot.GetName())

        # Call the planner.
        planning_traj = self.func(instance, planning_robot, *args, **kw_args)

        # Copy the trajectory back into the live environment to be safe.
        live_traj = openravepy.RaveCreateTrajectory(live_env, '')
        live_traj.Clone(planning_traj, 0)
        return live_traj

    def __get__(self, instance, instancetype):
        # Bind the self reference.
        return functools.partial(self.__call__, instance)

class Planner(object):
    methods = set()

    @classmethod
    def register_type(cls, method_name):

        def plan_wrapper(self, *args, **kw_args):
            return self.plan(method_name, args, kw_args)

        cls.methods.add(method_name)
        setattr(cls, method_name, plan_wrapper)

    def plan(self, method, args, kw_args):
        logger.info('Started planning with %s', self)
        try:
            method = getattr(self, method)
            return method(*args, **kw_args)
        except AttributeError:
            raise UnsupportedPlanningError
            

    @classmethod
    def bind(cls, instance, lazy_planner, executer=None):
        # The default executer is simply a wrapper for the planner.
        if executer is None:
            def executer(planning_method, args, kw_args):
                return planning_method(*args, **kw_args)

        def create_wrapper(method_name):
            # TODO: Copy __name__ and __doc__ from the implementation.
            def wrapper_method(*args, **kw_args):
                planner = lazy_planner()
                planning_method = getattr(planner, method_name)
                return executer(planning_method, args, kw_args)

            return wrapper_method

        for method_name in cls.methods:
            wrapper_method = create_wrapper(method_name)
            setattr(instance, method_name, wrapper_method)

class MetaPlanner(Planner):
    pass

class Sequence(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def __str__(self):
        return 'Sequence(%s)' % ', '.join(map(str, self._planners))

    def plan(self, method, args, kw_args):
        errors = list()

        for planner in self._planners:
            try:
                return planner.plan(method, args, kw_args)
            except MetaPlanningError as e:
                errors[planner] = e
            except PlanningError as e:
                logger.warning('Planning with %s failed: %s', planner, e)
                errors[planner] = e

        raise MetaPlanningError('All planners failed.', errors)

class Ranked(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def plan(self, method, args, kw_args):
        from threading import Condition, Thread

        planning_threads = list()
        results = [ None ] * len(self._planners)
        condition = Condition()

        # Start planning in parallel.
        for index, planner in enumerate(self._planners):
            def planning_thread(index, planner):
                try:
                    results[index] = planner.plan(method, args, kw_args)
                except PlanningError as e:
                    logger.warning('Planning with %s failed: %s', planner, e)
                    results[index] = e

                # Notify the main thread that we're done.
                condition.acquire()
                condition.notifyAll()
                condition.release()

            thread = Thread(target=planning_thread, args=(index, planner),
                            name='%s-planning' % planner)
            thread.daemon = True
            thread.start()

        # Block until a dominant planner finishes.
        traj = None
        condition.acquire()
        all_done = False

        while traj is None and not all_done:

            # TODO: Can we replace this with a fold?
            previous_done = True
            all_done = True #assume everyone is done
            for planner, result in zip(self._planners, results):
                if result is None:
                    previous_done = False
                    all_done = False
                elif isinstance(result, openravepy.Trajectory):
                    # All better planners have failed. Short-circuit and return this
                    # trajectory. It must be the highest-ranked one.
                    if previous_done:
                        traj = result
                        print 'Returning plan from %s' % planner
                        break
                elif not isinstance(result, PlanningError):
                    # TODO: Fill in the planning errors.
                    raise MetaPlanningError('Planner %s returned %r of type %r; '
                                            'expected a trajectory or PlanningError.'\
                                            % (planner, result, type(result)), list())
            

            # Wait for a planner to finish.
            if not all_done:
                condition.wait()

        condition.release()

        if traj is not None:
            return traj
        else:
            raise MetaPlanningError('All planners failed.', dict(zip(self._planners, results)))

class Fastest(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def plan(self, method, args, kw_args):
        raise NotImplemented
