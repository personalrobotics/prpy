#!/usr/bin/env python
import os
if os.environ.get('ROS_DISTRO', 'hydro')[0] in 'abcdef':
    import roslib; roslib.load_manifest('prpy')

import openravepy, unittest, numpy
from prpy.planning.chomp import DistanceFieldManager

class CHOMPModuleMock(object):
    def __init__(self, env):
        self.env = env
        self.computedistancefield_args = []
        self.removefield_args = []

    def GetEnv(self):
        return self.env

    def computedistancefield(self, kinbody=None, cube_extent=None,
                             aabb_padding=None, cache_filename=None,
                             releasegil=False, **kw_args):
        self.computedistancefield_args.append({
            'kinbody': kinbody,
            'cache_filename': cache_filename,
        })

    def removefield(self, kinbody=None, releasegil=False):
        self.removefield_args.append({
            'kinbody': kinbody,
        })

class DistanceFieldManagerTest(unittest.TestCase):
    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('data/wamtest2.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.body = self.env.GetKinBody('mug-table')
        self.bodies = set(self.env.GetBodies())

        self.module = CHOMPModuleMock(self.env)
        self.manager = DistanceFieldManager(self.module)

    def test_GetGeometricState_ChangeEnabledStatusChangesState(self):
        with self.env:
            link = self.robot.GetLinks()[0]
            
            link.Enable(True)
            state_before = self.manager.get_geometric_state(self.robot)

            link.Enable(False)
            state_after = self.manager.get_geometric_state(self.robot)

        self.assertNotEquals(state_after, state_before)

    def test_GetGeometricState_ChangeInDOFValuesChangesState(self):
        with self.env:
            lower_limits, upper_limits = self.robot.GetDOFLimits()

            self.robot.SetDOFValues(lower_limits)
            state_before = self.manager.get_geometric_state(self.robot)

            self.robot.SetDOFValues(upper_limits)
            state_after = self.manager.get_geometric_state(self.robot)

        self.assertNotEquals(state_after, state_before)

    def test_GetGeometricState_ChangeInKinematicsGeometryHashChangesState(self):
        self.robot.GetKinematicsGeometryHash = lambda: 'mock_before'
        state_before = self.manager.get_geometric_state(self.robot)

        self.robot.GetKinematicsGeometryHash = lambda: 'mock_after'
        state_after = self.manager.get_geometric_state(self.robot)

        self.assertNotEquals(state_after, state_before)

    def test_GetCachePath_DifferentStatesProduceDifferentPaths(self):
        self.robot.GetKinematicsGeometryHash = lambda: 'mock_before'
        state_before = self.manager.get_geometric_state(self.robot)
        path_before = self.manager.get_cache_path(state_before)

        self.robot.GetKinematicsGeometryHash = lambda: 'mock_after'
        state_after = self.manager.get_geometric_state(self.robot)
        path_after = self.manager.get_cache_path(state_after)

        self.assertNotEquals(path_after, path_before)

    def test_Sync_InitiallyCreatesAllDistanceFields(self):
        self.manager.sync(self.robot)

        computed_bodies = [ args['kinbody'] for args in self.module.computedistancefield_args ]
        self.assertItemsEqual(self.bodies, computed_bodies)
        self.assertEqual(len(self.module.removefield_args), 0)

    def test_Sync_SyncTwiceDoesNothing(self):
        self.manager.sync(self.robot)
        del self.module.computedistancefield_args[:]
        del self.module.removefield_args[:]
        self.manager.sync(self.robot)

        self.assertEqual(len(self.module.computedistancefield_args), 0)
        self.assertEqual(len(self.module.removefield_args), 0)

if __name__ == '__main__':
    unittest.main()
