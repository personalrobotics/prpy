#!/usr/bin/env python
import openravepy, unittest, numpy, numpy.testing
from openravepy import (Environment, IkFilterOptions, IkParameterization,
                        IkParameterizationType)
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.ompl import OMPLPlanner
from prpy.planning.base import PlanningError


# Generic test setup
#
# This class is the base class of all planner tests. It is responsible for
# setting up the environment, but does directly not implement any tests.
class BasePlannerTest(object):
    """Generic environment setup.
    """
    active_dof_indices = range(7)

    # Feasible start/goal pair.
    config_feasible_start = numpy.array([
         2.35061574,  0.61043555,  0.85      ,  1.80684444, -0.08639935,
        -0.69750474,  1.31656172
    ])
    config_feasible_goal = numpy.array([
        -0.84085883,  1.44573701,  0.2       ,  1.72620231, -0.81124757,
        -1.39363597,  1.29233111
    ])

    # This configuration is in collision with the environment, but is not in
    # self-collision.
    config_env_collision = numpy.array([
        3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
        2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
       -2.07807269e-01
    ])

    # This configuration is in self-collision, but is not in collision with the
    # environment.
    config_self_collision = numpy.array([
        2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
        2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
        0.00000000e+00
    ])

    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('data/wamtest2.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')

        # TODO: Planning should succeed with the floor present if CO_ActiveOnly
        # is set.
        self.env.Remove(self.env.GetKinBody('floor'))

        with self.env:
            self.robot.SetActiveManipulator(self.manipulator)
            self.robot.SetActiveDOFs(self.active_dof_indices)

        self.planner = self.planner_factory()

    def tearDown(self):
        self.env.Destroy()

    def _ValidatePath(self, path):
        self.assertEquals(path.GetEnv(), self.env)
        self.assertEquals(self.robot.GetActiveConfigurationSpecification(),
                          path.GetConfigurationSpecification())
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)

    def _CollisionCheckPath(self, traj):
        # NOTE: This assumes that the trajectory only contains joint_values.
        OpenStart = openravepy.Interval.OpenStart

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)

        # Check the first waypoint for collision. We do this outside of the
        # loop so we can run all collision checks with OpenEnd. This prevents
        # us from collision checking the intermediate waypoints twice.
        prev_waypoint = traj.GetWaypoint(0)
        check = params.CheckPathAllConstraints(prev_waypoint, prev_waypoint,
                                               [], [], 0., OpenStart)
        if check != 0:
            return True

        # Check the remainder of the path.
        for iwaypoint in xrange(1, traj.GetNumWaypoints() - 1):
            curr_waypoint = traj.GetWaypoint(iwaypoint)

            check = params.CheckPathAllConstraints(prev_waypoint, curr_waypoint,
                                                   [], [], 0., OpenStart)
            if check != 0:
                return True

            prev_waypoint = curr_waypoint

        return False

    def assertTransformClose(self, actual_pose, expected_pose,
                             linear_tol=1e-3, angular_tol=1e-3):
        rel_pose = numpy.dot(numpy.linalg.inv(actual_pose), expected_pose)
        distance = numpy.linalg.norm(rel_pose[0:3, 3])
        angle = numpy.arccos((numpy.trace(rel_pose[0:3, 0:3]) - 1.) / 2.)

        self.assertLessEqual(distance, linear_tol)
        self.assertLessEqual(angle, angular_tol)


# Method-specific tests
#
# Methods from BasePlannerTest are also available in these classes. However,
# they should NOT inherit from BasePlannerTest.
class PlanToConfigurationTests(object):
    def test_PlanToConfiguration_GoalIsFeasible_FindsSolution(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test
        path = self.planner.PlanToConfiguration(
            self.robot, self.config_feasible_goal)

        # Assert.
        self.assertEquals(path.GetEnv(), self.env)
        self.assertEquals(self.robot.GetActiveConfigurationSpecification(),
                          path.GetConfigurationSpecification())
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)
        first_waypoint = path.GetWaypoint(0)
        last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)

        self._ValidatePath(path)
        numpy.testing.assert_allclose(first_waypoint, self.config_feasible_start)
        numpy.testing.assert_allclose(last_waypoint, self.config_feasible_goal)
        self.assertFalse(self._CollisionCheckPath(path))

    def test_PlanToConfiguration_StartInCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            path = self.planner.PlanToConfiguration(
                self.robot, self.config_feasible_goal)

    def test_PlanToConfiguration_StartInSelfCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_self_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            path = self.planner.PlanToConfiguration(
                self.robot, self.config_feasible_goal)

    def test_PlanToConfiguration_GoalInCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            path = self.planner.PlanToConfiguration(
                self.robot, self.config_env_collision)

    def test_PlanToConfiguration_GoalInSelfCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            path = self.planner.PlanToConfiguration(
                self.robot, self.config_self_collision)


class PlanToEndEffectorPose(object):
    def test_PlanToEndEffectorPose_GoalIsFeasible_FindsSolution(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test
        path = self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

        # Assert
        self._ValidatePath(path)

        first_waypoint = path.GetWaypoint(0)
        numpy.testing.assert_allclose(first_waypoint, self.config_feasible_start)

        with self.env:
            last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)
            self.robot.SetActiveDOFValues(last_waypoint)
            last_ik = self.manipulator.GetEndEffectorTransform()

        self.assertTransformClose(last_ik, goal_ik)
        self.assertFalse(self._CollisionCheckPath(path))

    def test_PlanToEndEffectorPose_StartInCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_StartInSelfCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_self_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_GoalInCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_env_collision)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_GoalInSelfCollision_ThrowsPlanningError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_self_collision)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)


# Planner-specific tests
#
# Each of these classes MUST EXTEND BasePlannerTest, one or more
# method-specific test classes (e.g. PlanToConfigurationTests), and the
# unittest.TestCase class. The unittest.TestCase class MUST APPEAR LAST in the
# list of base classes.
"""
class OMPLPlannerTests(BasePlannerTest, PlanToConfigurationTests,
                      unittest.TestCase): 
    planner_factory = OMPLPlanner
"""

class CBiRRTPlannerTests(BasePlannerTest,
                         PlanToConfigurationTests,
                         PlanToEndEffectorPose,
                         unittest.TestCase): 
    planner_factory = CBiRRTPlanner

if __name__ == '__main__':
    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Warn)

    unittest.main()
