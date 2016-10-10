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
import numpy
import openravepy
from ..clone import Clone, CloneException
from ..futures import defer
from ..util import CopyTrajectory, GetTrajectoryTags, SetTrajectoryTags
from .exceptions import (ClonedPlanningError, MetaPlanningError,
                         PlanningError, UnsupportedPlanningError)

logger = logging.getLogger(__name__)


class Tags(object):
    SMOOTH = 'smooth'
    """
    The `SMOOTH` tag means waypoints are close enough together that we can
    approximate derivatives at the waypoints using divided differences. i.e.
    we can safely fit a spline without collision checking.
    """

    CONSTRAINED = 'constrained'
    """
    The `CONSTRAINED` tag means that the geometric path described by these
    waypoints respects a constraint.  This means that the path cannot be
    geometrically altered arbitrarily, at the risk of violating the original
    constraint, it should only be changed in timing.
    """

    PLANNER = 'planner'
    """
    The name of the planner used to generate a trajectory.
    """

    METHOD = 'planning_method'
    """
    The type of planning call used to generate a trajectory.
    """

    PLAN_TIME = 'planning_time'
    """
    The amount of time that was spent by a planner finding a solution.
    """

    POSTPROCESS_TIME = 'postprocess_time'
    """
    The amount of time that was spent modifying the trajectory for execution.
    """

    EXECUTION_TIME = 'execution_time'
    """
    The amount of time that was spent actually running a trajectory.
    """

    DETERMINISTIC_TRAJECTORY = 'deterministic'
    """
    Whether repeating the same planning query will produce the same trajectory.
    """

    DETERMINISTIC_ENDPOINT = 'deterministic_endpoint'
    """
    Whether repeating the same planning query will produce a trajectory with
    the same endpoint as this trajectory.
    """


class LockedPlanningMethod(object):
    """
    Decorate a planning method that locks the calling environment.
    """
    def __init__(self, func):
        self.func = func

    def __call__(self, instance, robot, *args, **kw_args):
        with robot.GetEnv():
            # Perform the actual planning operation.
            traj = self.func(instance, robot, *args, **kw_args)

            # Tag the trajectory with the planner and planning method
            # used to generate it. We don't overwrite these tags if
            # they already exist.
            tags = GetTrajectoryTags(traj)
            tags.setdefault(Tags.PLANNER, instance.__class__.__name__)
            tags.setdefault(Tags.METHOD, self.func.__name__)
            SetTrajectoryTags(traj, tags, append=False)

            return traj

    def __get__(self, instance, instancetype):
        # Bind the self reference and use update_wrapper to propagate the
        # function's metadata (e.g. name and docstring).
        wrapper = functools.partial(self.__call__, instance)
        functools.update_wrapper(wrapper, self.func)
        wrapper.is_planning_method = True
        return wrapper


class ClonedPlanningMethod(LockedPlanningMethod):
    """
    Decorate a planning method that clones the calling environment.
    """
    def __call__(self, instance, robot, *args, **kw_args):
        env = robot.GetEnv()

        # Store the original joint values and indices.
        joint_indices = [robot.GetActiveDOFIndices(), None]
        joint_values = [robot.GetActiveDOFValues(), None]

        try:
            with Clone(env, clone_env=instance.env) as cloned_env:
                cloned_robot = cloned_env.Cloned(robot)

                # Store the cloned joint values and indices.
                joint_indices[1] = cloned_robot.GetActiveDOFIndices()
                joint_values[1] = cloned_robot.GetActiveDOFValues()

                # Check for mismatches in the cloning and hackily reset them.
                # (This is due to a possible bug in OpenRAVE environment
                # cloning where in certain situations, the Active DOF ordering
                # and values do not match the parent environment.  It seems to
                # be exacerbated by multirotation joints, but the exact cause
                # and repeatability is unclear at this point.)
                if not numpy.array_equal(joint_indices[0], joint_indices[1]):
                    logger.warning(
                        "Cloned Active DOF index mismatch: %s != %s",
                        str(joint_indices[0]), str(joint_indices[1]))
                    cloned_robot.SetActiveDOFs(joint_indices[0])

                if not numpy.allclose(joint_values[0], joint_values[1]):
                    logger.warning(
                        "Cloned Active DOF value mismatch: %s != %s",
                        str(joint_values[0]), str(joint_values[1]))
                    cloned_robot.SetActiveDOFValues(joint_values[0])

                traj = super(ClonedPlanningMethod, self).__call__(
                    instance, cloned_robot, *args, **kw_args)
                return CopyTrajectory(traj, env=env)
        except CloneException as e:
            raise ClonedPlanningError(e)


class PlanningMethod(ClonedPlanningMethod):
    def __init__(self, func):
        logger.warn("Please explicitly declare a ClonedPlanningMethod "
                    "instead of using PlanningMethod.")
        super(ClonedPlanningMethod, self).__init__(func)


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
        super(BasePlanner, self).__init__()
        self.env = openravepy.Environment()


class MetaPlanner(Planner):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        super(MetaPlanner, self).__init__()
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
            return self.plan(method_name, args, kw_args)

        # Grab docstrings from the delegate planners.
        meta_wrapper.__name__ = method_name
        docstrings = list()
        for planner in self.get_planners_recursive(method_name):
            if planner.has_planning_method(method_name):
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
    KNOWN_KWARGS = set(['allow_nondeterministic'])

    def __init__(self, *planners, **kwargs):
        assert self.KNOWN_KWARGS.issuperset(kwargs.keys())

        super(Sequence, self).__init__()
        self._planners = planners
        self._allow_nondeterministic = kwargs.get(
            'allow_nondeterministic', False)

    def __str__(self):
        return 'Sequence({:s})'.format(', '.join(map(str, self._planners)))

    def get_planners(self, method_name):
        return [planner for planner in self._planners
                if planner.has_planning_method(method_name)]

    def plan(self, method, args, kw_args):
        from ..util import Timer

        errors = dict()
        is_sequence_deterministic = True

        for planner in self._planners:
            e = None

            try:
                if planner.has_planning_method(method):
                    logger.info('Sequence - Calling planner "%s".', str(planner))
                    planner_method = getattr(planner, method)

                    with Timer() as timer:
                        output = planner_method(*args, **kw_args)

                    if not is_sequence_deterministic:
                        # TODO: It is overly conservative to set _ENDPOINT,
                        # e.g. for PlanToConfiguration. Unfortunately, there is
                        # no easy way to detect this special case.
                        SetTrajectoryTags(output, {
                            Tags.DETERMINISTIC_TRAJECTORY: False,
                            Tags.DETERMINISTIC_ENDPOINT: False,
                        }, append=True)

                        if not self._allow_nondeterministic:
                            logger.warning(
                                'Tagging trajectory as non-deterministic because an'
                                ' earlier planner in the Sequence threw a'
                                ' non-deterministic PlanningError. Pass the'
                                ' "allow_nondeterministic" to this Sequence'
                                ' constructor if you intended this behavior.')

                    logger.info('Sequence - Planning succeeded after %.3f'
                                ' seconds with "%s".',
                                timer.get_duration(), str(planner))
                    return output
                else:
                    logger.debug('Sequence - Skipping planner "%s"; does not'
                                 ' have "%s" method.', str(planner), method)
            except MetaPlanningError as e:
                pass # Exception handled below.
            except PlanningError as e:
                logger.warning('Planning with %s failed: %s', planner, e)
                # Exception handled below.

            if e is not None:
                if e.deterministic is None:
                    is_sequence_deterministic = False
                    logger.warning(
                        'Planner %s raised a PlanningError without the'
                        ' "deterministic" flag set. Assuming the result'
                        ' is non-deterministic.', planner)
                elif not e.deterministic:
                    is_sequence_deterministic = False

                errors[planner] = e

        raise MetaPlanningError(
            'All planners failed.', errors, deterministic=is_sequence_deterministic)


class Ranked(MetaPlanner):
    def __init__(self, *planners):
        super(Ranked, self).__init__()
        self._planners = planners

    def __str__(self):
        return 'Ranked({0:s})'.format(', '.join(map(str, self._planners)))

    def get_planners(self, method_name):
        return [planner for planner in self._planners
                if planner.has_planning_method(method_name)]

    def plan(self, method, args, kw_args):
        all_planners = self._planners
        futures = []
        results = [None] * len(self._planners)

        # Helper function to call a planner and return its result.
        def call_planner(planner):
            planning_method = getattr(planner, method)
            return planning_method(*args, **kw_args)

        # Find only planners that support the required planning method.
        # Call every planners in parallel using a concurrent executor and
        # return the first non-error result in the ordering when available.
        for index, planner in enumerate(all_planners):
            if not planner.has_planning_method(method):
                results[index] = PlanningError(
                    "{:s} does not implement method {:s}."
                    .format(planner, method))
                continue
            else:
                futures.append((index, defer(call_planner, args=(planner,))))

        # Each time a planner completes, check if we have a valid result
        # (a planner found a solution and all higher-ranked planners had
        # already failed).
        for index, future in futures:
            try:
                return future.result()
            except MetaPlanningError as e:
                results[index] = e
            except PlanningError as e:
                logger.warning("Planning with {:s} failed: {:s}"
                               .format(planner, e))
                results[index] = e
        # TODO: if `cancel()` is supported, call it in a `finally` block here.

        raise MetaPlanningError("All planners failed.",
                                dict(zip(all_planners, results)))


class FirstSupported(MetaPlanner):
    def __init__(self, *planners):
        super(FirstSupported, self).__init__()
        self._planners = planners

    def __str__(self):
        return 'Fallback({:s})'.format(', '.join(map(str, self._planners)))

    def get_planners(self, method_name):
        return [planner for planner in self._planners
                if planner.has_planning_method(method_name)]

    def plan(self, method, args, kw_args):
        for planner in self._planners:
            if planner.has_planning_method(method):
                plan_fn = getattr(planner, method)

                try:
                    return plan_fn(*args, **kw_args)
                except UnsupportedPlanningError:
                    continue

        raise UnsupportedPlanningError()


class MethodMask(MetaPlanner):
    def __init__(self, planner, methods):
        super(MethodMask, self).__init__()
        self._methods = set(methods)
        self._planner = planner
        self._planners = [planner]

    def __str__(self):
        return 'Only({:s}, methods={:s})'.format(
            self._planner, list(self._methods))

    def get_planners(self, method_name):
        if method_name in self._methods:
            return [self._planner]
        else:
            return []

    def plan(self, method, args, kw_args):
        if method in self._methods:
            plan_fn = getattr(self._planner, method)
            return plan_fn(*args, **kw_args)
        else:
            raise UnsupportedPlanningError()
