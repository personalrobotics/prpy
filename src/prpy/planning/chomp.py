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

import collections
import contextlib
import logging
import numpy
import openravepy
from ..util import SetTrajectoryTags, GetLinearCollisionCheckPts
from ..collision import SimpleRobotCollisionChecker
from .exceptions import (
    CollisionPlanningError,
    SelfCollisionPlanningError
)
from base import (BasePlanner, PlanningError, UnsupportedPlanningError,
                  ClonedPlanningMethod, Tags)
from openravepy import CollisionOptions, CollisionOptionsStateSaver
from prpy.util import VanDerCorputSampleGenerator, SampleTimeGenerator
import prpy.tsr

SaveParameters = openravepy.KinBody.SaveParameters.LinkEnable

logger = logging.getLogger(__name__)

DistanceFieldKey = collections.namedtuple('DistanceFieldKey',
    [ 'kinematics_hash', 'enabled_mask', 'dof_values', 'dof_indices' ])


class DistanceFieldManager(object):
    def __init__(self, module, require_cache=False):
        self.module = module
        self.env = self.module.GetEnv()
        self.cache = dict()
        self.require_cache = require_cache

    def sync(self, robot):
        import os.path

        num_recomputed = 0

        for body in self.env.GetBodies():
            with body:
                # Only compute the SDF for links that are stationary. Other
                # links will be represented with spheres.
                if body == robot:
                    active_dof_indices = robot.GetActiveDOFIndices()
                    active_links = self.get_affected_links(robot, active_dof_indices)

                    for link in active_links:
                        link.Enable(False)

                body_name = body.GetName()
                current_state = self.get_geometric_state(body)
                logger.debug('Computed state for "%s": %s', body_name, current_state)

                # Check if the distance field is already loaded. Clear the
                # existing distance field if there is a key mismatch.
                cached_state = self.cache.get(body.GetName(), None)
                if cached_state is not None and cached_state != current_state:
                    logger.debug('Clearing distance field for "%s".', body_name)
                    self.module.removefield(body)
                    cached_state = None

                # Otherwise, compute a new distance field and save it to disk.
                if cached_state is None:
                    cache_path = self.get_cache_path(current_state)
                    logger.debug('Computing distance field for "%s"; filename: %s.',
                        body_name, os.path.basename(cache_path)
                    )

                    # Temporarily disable other bodies for distance field computation
                    other_bodies = []
                    for other_body in self.env.GetBodies():
                        if other_body == body:
                            continue
                        other_bodies.append(other_body)

                    other_savers = [
                        other_body.CreateKinBodyStateSaver(SaveParameters.LinkEnable)
                        for other_body in other_bodies]

                    with contextlib.nested(*other_savers):
                        for other_body in other_bodies:
                            other_body.Enable(False)
                        self.module.computedistancefield(body, cache_filename=cache_path, require_cache=self.require_cache)

                    self.cache[body_name] = current_state
                    num_recomputed += 1
                else:
                    logger.debug('Using existing distance field for "%s".', body_name)

        return num_recomputed

    @staticmethod
    def get_cache_path(state):
        import hashlib, pickle
        state_hash = hashlib.md5(pickle.dumps(state)).hexdigest()
        filename = 'chomp_{:s}.sdf'.format(state_hash)
        return openravepy.RaveFindDatabaseFile(filename, False)

    @staticmethod
    def get_geometric_state(body):
        enabled_mask = [ link.IsEnabled() for link in body.GetLinks() ]

        dof_indices = []
        for joint in body.GetJoints():

            for link in body.GetLinks():
                if not link.IsEnabled():
                    continue

                if body.DoesAffect(joint.GetJointIndex(), link.GetIndex()):
                    dof_indices += range(joint.GetDOFIndex(),
                                         joint.GetDOFIndex() + joint.GetDOF())
                    break

        return DistanceFieldKey(
            kinematics_hash = body.GetKinematicsGeometryHash(),
            enabled_mask = tuple(enabled_mask),
            dof_indices = tuple(dof_indices),
            # adding zero does -0.0 -> 0.0
            dof_values = tuple([round(v,5)+0 for v in body.GetDOFValues(dof_indices)]),
        )

    @staticmethod
    def get_affected_links(body, dof_indices):
        """Get the links that are affected by one or more active DOFs.
        """
        all_effected_links = set()

        for active_dof_index in dof_indices:
            joint = body.GetJointFromDOFIndex(active_dof_index)
            effected_links = [
                link for link in body.GetLinks() \
                if body.DoesAffect(joint.GetJointIndex(), link.GetIndex())
            ]
            all_effected_links.update(effected_links)

        return all_effected_links


class CHOMPPlanner(BasePlanner):
    def __init__(self, require_cache=False, robot_collision_checker=SimpleRobotCollisionChecker):
        super(CHOMPPlanner, self).__init__()
        self.require_cache = require_cache
        self.robot_collision_checker = robot_collision_checker
        self.setupEnv(self.env)

    def setupEnv(self, env):
        from types import MethodType

        self.env = env
        try:
            from orcdchomp import orcdchomp
            module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        except ImportError:
            raise UnsupportedPlanningError(
                'Unable to import "orcdchomp". Is or_cdchomp installed?')
        except openravepy.openrave_exception as e:
            raise UnsupportedPlanningError(
                'Failed loading "orcdchomp" module: ' + str(e))

        if module is None:
            raise UnsupportedPlanningError(
                'Failed loading "orcdchomp" module. Is or_cdchomp installed?')

        # This is a hack to prevent leaking memory.
        class CHOMPBindings(object):
            pass

        self.module = CHOMPBindings()
        self.module.module = module
        self.module.viewspheres =\
            MethodType(orcdchomp.viewspheres, module)
        self.module.computedistancefield =\
            MethodType(orcdchomp.computedistancefield, module)
        self.module.addfield_fromobsarray =\
            MethodType(orcdchomp.addfield_fromobsarray, module)
        self.module.removefield = MethodType(orcdchomp.removefield, module)
        self.module.create = MethodType(orcdchomp.create, module)
        self.module.iterate = MethodType(orcdchomp.iterate, module)
        self.module.gettraj = MethodType(orcdchomp.gettraj, module)
        self.module.destroy = MethodType(orcdchomp.destroy, module)
        self.module.runchomp = MethodType(orcdchomp.runchomp, module)
        self.module.GetEnv = self.module.module.GetEnv

        # Create a DistanceFieldManager to track which distance fields are
        # currently loaded.
        self.distance_fields = DistanceFieldManager(
            self.module, require_cache=self.require_cache)

    def __str__(self):
        return 'CHOMP'

    def ComputeDistanceField(self, robot):
        logger.warning('ComputeDistanceField is deprecated. Distance fields are'
                       ' now implicity created by DistanceFieldManager.')

    @ClonedPlanningMethod
    def OptimizeTrajectory(self, robot, traj, lambda_=100.0, n_iter=50,
                           **kw_args):
        self.distance_fields.sync(robot)

        cspec = traj.GetConfigurationSpecification()
        cspec.AddDeltaTimeGroup()
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)

        for i in xrange(traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            cspec.InsertDeltaTime(waypoint, .1)
            traj.Insert(i, waypoint, True)
        
        return self._Plan(robot=robot, starttraj=traj, lambda_=lambda_,
            n_iter=n_iter, **kw_args)

    @ClonedPlanningMethod
    def PlanToConfiguration(self, robot, goal, lambda_=100.0, n_iter=15,
                            **kwargs):
        """
        Plan to a single configuration with single-goal CHOMP.
        @param robot
        @param goal goal configuration
        @param lambda_ step size
        @param n_iter number of iterations
        """
        self.distance_fields.sync(robot)

        return self._Plan(robot=robot, adofgoal=goal, lambda_=lambda_,
            n_iter=n_iter, **kwargs)

    def _Plan(self, robot, sampling_func=VanDerCorputSampleGenerator, **kwargs):
        try:
            # Disable collision checking since we will perform them below.
            traj = self.module.runchomp(robot=robot, no_collision_check=True,
                    releasegil=True, **kwargs)
        except Exception as e:
            raise PlanningError(str(e))

        # Strip the extra groups added by CHOMP and change the trajectory to be
        # linearly interpolated, as required by GetLinearCollisionCheckPts.
        cspec = robot.GetActiveConfigurationSpecification('linear')
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)

        # Collision check the trajectory at DOF resolution. We do this in
        # Python, instead of using CHOMP's native support for this, so we have
        # access to the CollisionReport object.
        checks = GetLinearCollisionCheckPts(robot, traj, norm_order=2,
            sampling_func=sampling_func)

        with CollisionOptionsStateSaver(self.env.GetCollisionChecker(),
                                        CollisionOptions.ActiveDOFs):

            # Instantiate a robot checker
            robot_checker = self.robot_collision_checker(robot)

            for t, q in checks:
                robot.SetActiveDOFValues(q)

                # Check collision (throws an exception on collision)
                robot_checker.VerifyCollisionFree()

        SetTrajectoryTags(traj, {Tags.SMOOTH: True}, append=True)

        return traj
