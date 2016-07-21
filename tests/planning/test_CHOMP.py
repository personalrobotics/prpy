from methods.PlanToConfiguration import (
    PlanToConfigurationCompleteTest,
    PlanToConfigurationStraightLineTest,
    PlanToConfigurationTest,
    PlanToConfigurationTestCollisionTest,
)
from prpy.planning.chomp import CHOMPPlanner
from planning_helpers import BasePlannerTest
from unittest import TestCase


class CHOMPModuleMock(object):
    def __init__(self, env):
        self.env = env
        self.sequence = 0
        self.computedistancefield_args = []
        self.removefield_args = []

    def GetEnv(self):
        return self.env

    def computedistancefield(self, kinbody=None, cube_extent=None,
                             aabb_padding=None, cache_filename=None,
                             releasegil=False, **kw_args):
        self.computedistancefield_args.append({
            '__sequence__': self.sequence,
            'kinbody': kinbody,
            'cache_filename': cache_filename,
        })
        self.sequence += 1

    def removefield(self, kinbody=None, releasegil=False):
        self.removefield_args.append({
            '__sequence__': self.sequence,
            'kinbody': kinbody,
        })
        self.sequence += 1


class DistanceFieldManagerTest(TestCase):
    def setUp(self):
        from prpy.planning.chomp import DistanceFieldManager
        from openravepy import Environment

        self.env = Environment()
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

    def test_Sync_IgnoresActiveRobotLinks(self):
        self.is_processed = False

        def computedistancefield(kinbody=None, cube_extent=None,
                                 aabb_padding=None, cache_filename=None,
                                 releasegil=False, **kw_args):
            if kinbody.GetName() == self.robot.GetName():
                self.assertTrue(self.robot.GetLink('segway').IsEnabled())
                self.assertTrue(self.robot.GetLink('wam0').IsEnabled())
                self.assertTrue(self.robot.GetLink('wam1').IsEnabled())
                self.assertTrue(self.robot.GetLink('wam2').IsEnabled())
                self.assertTrue(self.robot.GetLink('wam3').IsEnabled())
                self.assertFalse(self.robot.GetLink('wam4').IsEnabled())
                self.assertFalse(self.robot.GetLink('wam5').IsEnabled())
                self.assertFalse(self.robot.GetLink('wam6').IsEnabled())
                self.assertFalse(self.robot.GetLink('wam7').IsEnabled())
                self.assertFalse(self.robot.GetLink('handbase').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger0-0').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger0-1').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger0-2').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger1-0').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger1-1').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger1-2').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger2-1').IsEnabled())
                self.assertFalse(self.robot.GetLink('Finger2-2').IsEnabled())
                self.is_processed = True

        dof_index = self.robot.GetJoint('Elbow').GetDOFIndex()
        self.robot.SetActiveDOFs([ dof_index ])
        self.robot.Enable(True)

        self.module.computedistancefield = computedistancefield
        self.manager.sync(self.robot)

        self.assertTrue(self.is_processed)

    def test_Sync_RecomputesDistanceFieldIfStateChanges(self):
        self.manager.sync(self.robot)
        del self.module.computedistancefield_args[:]
        del self.module.removefield_args[:]

        # Change the geometry to invalidate the key.
        link = self.robot.GetLink('segway')
        link.SetGroupGeometries('test', [])
        link.SetGeometriesFromGroup('test')

        self.manager.sync(self.robot)

        self.assertEqual(len(self.module.computedistancefield_args), 1)
        self.assertEqual(self.module.computedistancefield_args[0]['kinbody'], self.robot)

        self.assertEqual(len(self.module.removefield_args), 1)
        self.assertEqual(self.module.removefield_args[0]['kinbody'], self.robot)
        self.assertLess(self.module.removefield_args[0]['__sequence__'],
                        self.module.computedistancefield_args[0]['__sequence__'])


class CHOMPPlannerTest(BasePlannerTest,
                       PlanToConfigurationTest,
                       PlanToConfigurationCompleteTest,
                       # TODO: This should be possible to implement in CHOMP.
                       #PlanToConfigurationTestCollisionTest,
                       PlanToConfigurationStraightLineTest,
                       TestCase):
    planner_factory = CHOMPPlanner

    def setUp(self):
        super(CHOMPPlannerTest, self).setUp()
