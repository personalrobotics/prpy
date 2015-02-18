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

import abc
import functools
import logging
import openravepy
from ..clone import Clone
from ..util import CopyTrajectory

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

    def __call__(self, instance, robot, *args, **kw_args):
        env = robot.GetEnv()
        defer = kw_args.get('defer')

        with Clone(env, clone_env=instance.env,
                   lock=True, unlock=False) as cloned_env:
            cloned_robot = Cloned(robot)

            def call_planner():
                try:
                    planner_traj = self.func(instance, cloned_robot,
                                             *args, **kw_args)
                    return CopyTrajectory(planner_traj, env=env)
                finally:
                    cloned_env.Unlock()

            if defer is True:
                from trollius.executor import get_default_executor
                executor = kw_args.get('executor') or get_default_executor()
                return executor.submit(call_planner)
            else:
                return call_planner()

    def __get__(self, instance, instancetype):
        # Bind the self reference and use update_wrapper to propagate the
        # function's metadata (e.g. name and docstring).
        wrapper = functools.partial(self.__call__, instance)
        functools.update_wrapper(wrapper, self.func)
        wrapper.is_planning_method = True
        return wrapper


class Planner(object):
    def has_planning_method(self, method_name):
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            if hasattr(method, 'is_planning_method'):
                return method.is_planning_method
            else:
                return False
        else:
            return False

    def get_planning_method_names(self):
        return filter(lambda method_name: self.has_planning_method(method_name), dir(self))

class BasePlanner(Planner):
    def __init__(self):
        self.env = openravepy.Environment()

class MetaPlanner(Planner):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self._planners = list()

    def has_planning_method(self, method_name):
        for planner in self._planners:
            if planner.has_planning_method(method_name):
                return True

        return False

    def get_planning_method_names(self):
        method_names = set()
        for planner in self._planners:
            method_names.update(planner.get_planning_method_names())

        return list(method_names)

    @abc.abstractmethod
    def get_planners(self, method_name):
        pass

    def get_planners_recursive(self, method):
        all_planners = set()

        for planner in self.get_planners(method):
            if isinstance(planner, MetaPlanner):
                sub_planners = planner.get_planners_recursive(method)
                all_planners = all_planners.union(sub_planners)
            else:
                all_planners.add(planner)

        return list(all_planners)

    def __dir__(self):
        return self.get_planning_method_names()

    def __getattr__(self, method_name):
        if not self.has_planning_method(method_name):
            raise AttributeError("Object {:s} has no attribute '{:s}'.".format(
                                 repr(self), method_name))

        def meta_wrapper(*args, **kw_args):
            defer = kw_args.get('defer')

            if defer is True:
                from trollius.executor import get_default_executor
                executor = kw_args.get('executor') or get_default_executor()
                return executor.submit(self.plan, method_name, args, kw_args)
            else:
                return self.plan(method_name, args, kw_args)

        # Grab docstrings from the delegate planners.
        meta_wrapper.__name__ = method_name
        docstrings = list()
        for planner in self.get_planners_recursive(method_name):
            if hasattr(planner, method_name):
                planner_method = getattr(planner, method_name)
                docstrings.append((planner, planner_method))

        # Concatenate the docstrings.
        if docstrings:
            meta_wrapper.__doc__ = ''

            for planner, planner_method in docstrings:
                formatted_docstring = ''

                if isinstance(planner, MetaPlanner):
                    if planner_method.__doc__ is not None:
                        formatted_docstring += planner_method.__doc__
                else:
                    # Header for this planner.
                    formatted_docstring += str(planner) + ': ' + planner_method.__name__ + '\n'
                    formatted_docstring += '-' * (len(formatted_docstring) - 1) + '\n'

                    # Docstring.
                    if planner_method.__doc__ is not None:
                        formatted_docstring += planner_method.__doc__
                    else:
                        formatted_docstring += '<no docstring>\n'

                    # Blank line.
                    formatted_docstring += '\n'

                meta_wrapper.__doc__ += formatted_docstring

        return meta_wrapper


class Sequence(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def __str__(self):
        return 'Sequence({0:s})'.format(', '.join(map(str, self._planners)))

    def get_planners(self, method_name):
        return [planner for planner in self._planners
                if hasattr(planner, method_name)]

    def plan(self, method, args, kw_args):
        errors = dict()

        for planner in self._planners:
            try:
                if hasattr(planner, method):
                    logger.debug('Sequence - Calling planner "%s".', str(planner))
                    planner_method = getattr(planner, method)
                    kw_args['defer'] = False
                    return planner_method(*args, **kw_args)
                else:
                    logger.debug('Sequence - Skipping planner "%s"; does not have "%s" method.',
                                 str(planner), method)
            except MetaPlanningError as e:
                errors[planner] = e
            except PlanningError as e:
                logger.warning('Planning with %s failed: %s', planner, e)
                errors[planner] = e

        raise MetaPlanningError('All planners failed.', errors)


class Ranked(MetaPlanner):
    def __init__(self, *planners):
        self._planners = planners

    def __str__(self):
        return 'Ranked({0:s})'.format(', '.join(map(str, self._planners)))

    def get_planners(self, method_name):
        return [planner for planner in self._planners
                if hasattr(planner, method_name)]

    def plan(self, method, args, kw_args):
        all_planners = self._planners
        planners = []
        results = [None] * len(self._planners)

        # Find only planners that support the required planning method.
        for index, planner in enumerate(all_planners):
            if not hasattr(planner, method):
                results[index] = PlanningError(
                    "{:s} does not implement method {:s}."
                    .format(planner, method)
                )
                continue
            else:
                planners.append((index, planner))

        # Helper function to call a planner and store its result or error.
        def call_planner(index, planner):
            try:
                planning_method = getattr(planner, method)
                kw_args['defer'] = False
                results[index] = planning_method(*args, **kw_args)
            except MetaPlanningError as e:
                results[index] = e
            except PlanningError as e:
                logger.warning("Planning with {:s} failed: {:s}"
                               .format(planner, e))
                results[index] = e

        # Call every planners in parallel using a concurrent executor and
        # return the first non-error result in the ordering when available.
        from trollius.executor import get_default_executor
        from trollius.tasks import as_completed

        executor = kw_args.get('executor') or get_default_executor()
        futures = [executor.submit(call_planner, planner)
                   for planner in planners]

        # Each time a planner completes, check if we have a valid result
        # (a planner found a solution and all higher-ranked planners had
        # already failed).
        for _ in as_completed(futures):
            for result in results:
                if result is None:
                    break
                elif isinstance(result, openravepy.Trajectory):
                    return result
                elif not isinstance(result, PlanningError):
                    logger.warning(
                        "Planner {:s} returned {} of type {}; "
                        "expected a trajectory or PlanningError."
                        .format(str(planner), result, type(result))
                    )

        raise MetaPlanningError("All planners failed.",
                                dict(zip(all_planners, results)))
