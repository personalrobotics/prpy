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

import contextlib, collections, logging, numpy, openravepy, rospkg, warnings
import prpy.tsr
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

logger = logging.getLogger('prpy.planning.chomp')

DistanceFieldKey = collections.namedtuple('DistanceFieldKey',
    [ 'kinematics_hash', 'enabled_mask', 'dof_values', 'dof_indices' ])

class DistanceFieldManager(object):
    def __init__(self, module):
        self.module = module
        self.env = self.module.GetEnv()
        self.cache = dict()

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
                    logger.debug('Clearing distance field for "{:s}".', body_name)
                    self.module.removefield(body)
                    cached_state = None

                # Otherwise, compute a new distance field and save it to disk.
                if cached_state is None:
                    cache_path = self.get_cache_path(current_state)
                    logger.debug('Computing distance field for "%s"; filename: %s.',
                        body_name, os.path.basename(cache_path)
                    )

                    self.module.computedistancefield(body, cache_filename=cache_path)
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
            dof_values = tuple(body.GetDOFValues(dof_indices)),
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
    def __init__(self):
        super(CHOMPPlanner, self).__init__()
        self.setupEnv(self.env)

    def setupEnv(self, env):
        self.env = env
        try:
            from orcdchomp import orcdchomp
            module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        except ImportError:
            raise UnsupportedPlanningError('Unable to import orcdchomp.')
        except openravepy.openrave_exception as e:
            raise UnsupportedPlanningError('Unable to create orcdchomp module: %s' % e)

        if module is None:
            raise UnsupportedPlanningError('Failed loading module.')
        self.initialized = False
        #orcdchomp.bind(self.module)

        class Object():
            pass

        import types

        self.module = Object()
        self.module.module = module
        self.module.viewspheres = types.MethodType(orcdchomp.viewspheres,module)
        self.module.computedistancefield = types.MethodType(orcdchomp.computedistancefield,module)
        self.module.addfield_fromobsarray = types.MethodType(orcdchomp.addfield_fromobsarray,module)
        self.module.removefield = types.MethodType(orcdchomp.removefield,module)
        self.module.create = types.MethodType(orcdchomp.create,module)
        self.module.iterate = types.MethodType(orcdchomp.iterate,module)
        self.module.gettraj = types.MethodType(orcdchomp.gettraj,module)
        self.module.destroy = types.MethodType(orcdchomp.destroy,module)
        self.module.runchomp = types.MethodType(orcdchomp.runchomp,module)
        self.module.GetEnv = self.module.module.GetEnv

        self.distance_fields = DistanceFieldManager(self.module)

    def __str__(self):
        return 'CHOMP'

    @PlanningMethod
    def OptimizeTrajectory(self, robot, traj, lambda_=100.0, n_iter=50, **kw_args):
        self.distance_fields.sync(robot)

        cspec = traj.GetConfigurationSpecification()
        cspec.AddDeltaTimeGroup()
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)
        for i in xrange(traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            cspec.InsertDeltaTime(waypoint, .1)
            traj.Insert(i, waypoint, True)
        
        run = self.module.create(robot=robot, starttraj=traj, lambda_=lambda_)
        self.module.iterate(n_iter=n_iter, run=run)
        self.module.destroy(run=run)
        return traj

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, lambda_=100.0, n_iter=15, **kw_args):
        """
        Plan to a single configuration with single-goal CHOMP.
        @param robot
        @param goal goal configuration
        @param lambda_ step size
        @param n_iter number of iterations
        """
        self.distance_fields.sync(robot)

        try:
            return self.module.runchomp(robot=robot, adofgoal=goal,
                                        lambda_=lambda_, n_iter=n_iter,
                                        releasegil=True, **kw_args)
        except Exception as e:
            raise PlanningError(str(e))

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, lambda_=100.0, n_iter=100, goal_tolerance=0.01, **kw_args):
        """
        Plan to a desired end-effector pose using GSCHOMP
        @param robot
        @param goal_pose desired end-effector pose
        @param lambda_ step size
        @param n_iter number of iterations
        @param goal_tolerance tolerance in meters
        @return traj
        """
        self.distance_fields.sync(robot)

        # CHOMP only supports start sets. Instead, we plan backwards from the
        # goal TSR to the starting configuration. Afterwards, we reverse the
        # trajectory.
        manipulator_index = robot.GetActiveManipulatorIndex()
        goal_tsr = prpy.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        start_config = robot.GetActiveDOFValues()

        try:
            traj = self.module.runchomp(robot=robot, adofgoal=start_config, start_tsr=goal_tsr,
                                        lambda_=lambda_, n_iter=n_iter, goal_tolerance=goal_tolerance,
                                        releasegil=True, **kw_args)
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
            raise PlanningError('CHOMP deviated from the goal pose by {0:f} meters.'.format(goal_distance))

        return traj

    '''
    JK - Disabling. This is not working reliably.
    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, lambda_=100.0, n_iter=100, goal_tolerance=0.01, **kw_args):
        """
        Plan to a goal TSR.
        @param robot
        @param tsrchains A TSR chain with a single goal tsr
        @return traj
        """
        self.distance_fields.sync(robot)

        manipulator_index = robot.GetActiveManipulatorIndex()
        start_config = robot.GetActiveDOFValues()
        
        if len(tsrchains) != 1:
            raise UnsupportedPlanningError('CHOMP')
        
        tsrchain = tsrchains[0]
        if not tsrchain.sample_goal or len(tsrchain.TSRs) > 1:
            raise UnsupportedPlanningError('CHOMP: TSR chain must contain a single goal tsr')
        
        try:
            goal_tsr = tsrchain.TSRs[0]
            traj = self.module.runchomp(robot=robot, adofgoal=start_config, start_tsr=goal_tsr,
                                        lambda_=lambda_, n_iter=n_iter, goal_tolerance=goal_tolerance,
                                        releasegil=True, **kw_args)

            traj = openravepy.planningutils.ReverseTrajectory(traj)

        except RuntimeError, e:
            raise PlanningError(str(e))

        return traj
    '''

    def ComputeDistanceField(self, robot):
        logger.warning('ComputeDistanceField is deprecated. Distance fields are'
                       ' now implicity created by DistanceFieldManager.')

