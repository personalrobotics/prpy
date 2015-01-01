#!/usr/bin/env python
import numpy
import openravepy
import unittest
from prpy.planning.base import PlanningError
from prpy.planning.snap import SnapPlanner

class SnapPlannerTest(unittest.TestCase):
    config_env_collision = numpy.array([
        3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
        2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
       -2.07807269e-01
    ])
    config_self_collision = numpy.array([
        2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
        2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
        0.00000000e+00
    ])
    config_segment1 = numpy.array([
        1.23760308e-01,   6.05769772e-01,  -5.00000000e-02,
        1.33403907e+00,   4.47724617e-01,  -3.14811779e-01,
       -1.90265540e+00
    ])
    config_segment2 = numpy.array([
        1.12376031e+00,   6.05769772e-01,  -5.00000000e-02,
        1.33403907e+00,   4.47724617e-01,  -3.14811779e-01,
       -1.90265540e+00
    ])
    
    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('data/wamtest2.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.body = self.env.GetKinBody('mug-table')
        self.bodies = set(self.env.GetBodies())

        self.planner = SnapPlanner()

        with self.env:
            manipulator = self.robot.GetManipulator('arm')
            self.robot.SetActiveManipulator(manipulator)
            self.robot.SetActiveDOFs(manipulator.GetArmIndices())

            # The Segway is in contact with the floor, which can cause some
            # problems with planning.
            self.env.Remove(self.env.GetKinBody('floor'))

            # Compute a small motion that is larger than the collision checking
            # radius. This will be used to test endpoint conditions.
            dof_resolutions = self.robot.GetActiveDOFResolutions()
            self.config_epsilon = numpy.zeros(len(dof_resolutions))
            self.config_epsilon[0] += 2 * dof_resolutions[0]

    def test_PlanToConfiguration_StartInCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_env_collision)
        self.assertTrue(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test and assert.
        goal_config = self.config_env_collision + self.config_epsilon

        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, goal_config)

    def test_PlanToConfiguration_StartInSelfCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_self_collision)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertTrue(self.robot.CheckSelfCollision())

        # Test and assert.
        goal_config = self.config_self_collision + self.config_epsilon

        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, goal_config)

    def test_PlanToConfiguration_SegmentInCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_segment2)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        self.robot.SetActiveDOFValues(self.config_segment1)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test and assert.
        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, self.config_segment2)

if __name__ == '__main__':
    unittest.main()
