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

import logging, functools, openravepy
import time
from .. import ik_ranking

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
        """
        Wrapper __call__.
        """
        live_env = live_robot.GetEnv()
        planning_env = instance.env

        if live_env != planning_env:
            # Clone the live environment for planning.
            from openravepy import CloningOptions
            planning_env.Clone(live_env, CloningOptions.Bodies)
            planning_robot = planning_env.GetRobot(live_robot.GetName())
        else:
            planning_robot = live_robot

        # Call the planner.
        planning_traj = self.func(instance, planning_robot, *args, **kw_args)

        # Copy the trajectory back into the live environment to be safe.
        if live_env != planning_env:
            live_traj = openravepy.RaveCreateTrajectory(live_env, '')
            live_traj.Clone(planning_traj, 0)
        else:
            live_traj = planning_traj

        return live_traj

    def __get__(self, instance, instancetype):
        # Bind the self reference and use update_wrapper to propagate the
        # function's metadata.
        wrapper = functools.partial(self.__call__, instance)
        wrapper.__doc__ = 'Wrapper __call__.'
        functools.update_wrapper(wrapper, self.__call__)
        return wrapper

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

class BasePlanner(Planner):
    @PlanningMethod
    def PlanToIK(self, robot, goal_pose, ranker=ik_ranking.JointLimitAvoidance, num_attempts=1, **kw_args):
        import numpy
        from openravepy import IkFilterOptions, IkParameterization, IkParameterizationType

        # FIXME: Currently meta-planners duplicate IK ranking in each planning
        # thread. It should be possible to fix this by IK ranking once, then
        # calling PlanToConfiguration in separate threads.

        # Find an unordered list of IK solutions.
        with robot.GetEnv():
            manipulator = robot.GetActiveManipulator()
            ik_param = IkParameterization(goal_pose, IkParameterizationType.Transform6D)
            ik_solutions = manipulator.FindIKSolutions(ik_param, IkFilterOptions.CheckEnvCollisions)

        if ik_solutions.shape[0] == 0:
            raise PlanningError('There is no IK solution at the goal pose.')

        # Sort the IK solutions in ascending order by the costs returned by the
        # ranker. Lower cost solutions are better and infinite cost solutions are
        # assumed to be infeasible.
        scores = ranker(robot, ik_solutions)
        sorted_indices = numpy.argsort(scores)
        sorted_indices = sorted_indices[~numpy.isposinf(scores)]
        scores = scores[~numpy.isposinf(scores)]
        sorted_ik_solutions = ik_solutions[sorted_indices, :]

        if sorted_ik_solutions.shape[0] == 0:
            raise PlanningError('All IK solutions have infinite cost.')

        # Sequentially plan to the solutions in descending order of cost.
        num_attempts = min(sorted_ik_solutions.shape[0], num_attempts)
        for i, ik_solution in enumerate(sorted_ik_solutions[0:num_attempts, :]):
            try:
                traj = self.PlanToConfiguration(robot, ik_solution)
                logger.info('Planned to IK solution %d of %d.', i + 1, num_attempts)
                return traj
            except PlanningError as e:
                logger.warning('Planning to IK solution %d of %d failed: %s',
                               i + 1, num_attempts, e)

        raise PlanningError('Planning to the top {0:d} of {1:d} IK solutions failed.'.format(
                            num_attempts, sorted_ik_solutions.shape[0]))

class MetaPlanner(Planner):
    pass

class Sequence(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def __str__(self):
        return 'Sequence(%s)' % ', '.join(map(str, self._planners))

    def plan(self, method, args, kw_args):
        errors = dict()

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
                except Exception as e:
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

        # TODO: Can we replace this with a fold?
        while traj is None and not all_done:
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
