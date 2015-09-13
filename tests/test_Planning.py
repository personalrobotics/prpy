#!/usr/bin/env python
import openravepy
import unittest
import numpy
from openravepy import Environment
from prpy.clone import CloneException
from prpy.planning.base import PlanningError
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.ompl import OMPLPlanner, OMPLSimplifier
from prpy.planning.retimer import ParabolicRetimer
from prpy.planning.mac_smoother import MacSmoother
from numpy.testing import assert_allclose


openravepy.RaveInitialize(True)
openravepy.misc.InitOpenRAVELogging()
openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)


VerifyTrajectory = openravepy.planningutils.VerifyTrajectory


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
        +2.35061574,  0.61043555,  0.85000000,  1.80684444, -0.08639935,
        -0.69750474,  1.31656172
    ])
    config_feasible_goal = numpy.array([
        -0.84085883,  1.44573701,  0.20000000,  1.72620231, -0.81124757,
        -1.39363597,  1.29233111
    ])

    # This configuration is in collision with the environment, but is not in
    # self-collision.
    config_env_collision = numpy.array([
        +3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
        +2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
        -2.07807269e-01
    ])

    # This configuration is in self-collision, but is not in collision with the
    # environment.
    config_self_collision = numpy.array([
        2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
        2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
        0.00000000e+00
    ])

    # Waypoints that can be pair-wise connected by straight line paths.
    waypoint1 = numpy.array([
        1.12376031,  0.60576977, -0.05000000,
        1.33403907,  0.44772461, -0.31481177,
        1.90265540
    ])
    waypoint2 = numpy.array([
        1.53181533,  0.80270404, -0.05000000,
        1.75341989,  0.21348846, -0.91026757,
        1.59603932
    ])
    waypoint3 = numpy.array([
        1.50376031,  0.60576977, -0.05000000,
        1.33403907,  0.44772461, -0.31481177,
        1.90265540
    ])

    def setUp(self):
        self.env = Environment()
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
        self.assertEquals(self.robot.GetActiveConfigurationSpecification('linear'),
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

            check = params.CheckPathAllConstraints(
                prev_waypoint, curr_waypoint, [], [], 0., OpenStart)
            if check != 0:
                return True

            prev_waypoint = curr_waypoint

        return False

    def _ComputeArcLength(self, traj):
        distance = 0.

        for iwaypoint in xrange(1, traj.GetNumWaypoints()):
            prev_waypoint = traj.GetWaypoint(iwaypoint - 1)
            curr_waypoint = traj.GetWaypoint(iwaypoint)
            distance += numpy.linalg.norm(curr_waypoint - prev_waypoint)

        return distance

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
class PlanToConfigurationTest(object):
    def test_PlanToConfiguration_GoalIsFeasible_FindsSolution(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test
        path = self.planner.PlanToConfiguration(
            self.robot, self.config_feasible_goal)

        # Assert.
        self.assertEquals(path.GetEnv(), self.env)
        self.assertEquals(self.robot.GetActiveConfigurationSpecification('linear'),
                          path.GetConfigurationSpecification())
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)
        first_waypoint = path.GetWaypoint(0)
        last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)

        self._ValidatePath(path)
        assert_allclose(first_waypoint, self.config_feasible_start)
        assert_allclose(last_waypoint, self.config_feasible_goal)
        self.assertFalse(self._CollisionCheckPath(path))

    def test_PlanToConfiguration_StartInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(
                self.robot, self.config_feasible_goal)

    def test_PlanToConfiguration_StartInSelfCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_self_collision)

        # Test/Assert
        with self.assertRaises((PlanningError, CloneException)):
            self.planner.PlanToConfiguration(
                self.robot, self.config_feasible_goal)

    def test_PlanToConfiguration_GoalInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(
                self.robot, self.config_env_collision)

    def test_PlanToConfiguration_GoalInSelfCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(
                self.robot, self.config_self_collision)


class PlanToEndEffectorPoseTest(object):
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
        assert_allclose(first_waypoint, self.config_feasible_start)

        with self.env:
            last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)
            self.robot.SetActiveDOFValues(last_waypoint)
            last_ik = self.manipulator.GetEndEffectorTransform()

        self.assertTransformClose(last_ik, goal_ik)
        self.assertFalse(self._CollisionCheckPath(path))

    def test_PlanToEndEffectorPose_StartInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_StartInSelfCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_self_collision)

        # Test/Assert
        with self.assertRaises((PlanningError, CloneException)):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_GoalInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_env_collision)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

    def test_PlanToEndEffectorPose_GoalInSelfCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_self_collision)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorPose(self.robot, goal_ik)


class ShortcutPathTest(object):
    def setUp(self):
        self.input_path = openravepy.RaveCreateTrajectory(self.env, '')
        self.input_path.Init(self.robot.GetActiveConfigurationSpecification('linear'))
        self.input_path.Insert(0, self.waypoint1)
        self.input_path.Insert(1, self.waypoint2)
        self.input_path.Insert(2, self.waypoint3)

    def test_ShortcutPath_ShortcutExists_ReducesLength(self):
        # Setup/Test
        smoothed_path = self.planner.ShortcutPath(self.robot, self.input_path)

        # Assert
        self.assertEquals(smoothed_path.GetConfigurationSpecification(),
                          self.input_path.GetConfigurationSpecification())
        self.assertGreaterEqual(smoothed_path.GetNumWaypoints(), 2)

        n = smoothed_path.GetNumWaypoints()
        assert_allclose(smoothed_path.GetWaypoint(0),     self.waypoint1)
        assert_allclose(smoothed_path.GetWaypoint(n - 1), self.waypoint3)

        self.assertLess(self._ComputeArcLength(smoothed_path),
                        0.5 * self._ComputeArcLength(self.input_path))

    # TODO: Test some of the error cases.


class SmoothTrajectoryTest(object):
    def setUp(self):
        cspec = self.robot.GetActiveConfigurationSpecification('linear')

        self.feasible_path = openravepy.RaveCreateTrajectory(self.env, '')
        self.feasible_path.Init(cspec)
        self.feasible_path.Insert(0, self.waypoint1)
        self.feasible_path.Insert(1, self.waypoint2)
        self.feasible_path.Insert(2, self.waypoint3)

    def test_SmoothTrajectory_DoesNotModifyStartPoint(self):
        # Setup/Test
        traj = self.planner.RetimeTrajectory(self.robot, self.feasible_path)

        # Assert
        cspec = self.robot.GetActiveConfigurationSpecification('linear')
        self.assertGreaterEqual(traj.GetNumWaypoints(), 2)

        first_waypoint = traj.GetWaypoint(0, cspec)
        last_waypoint = traj.GetWaypoint(traj.GetNumWaypoints() - 1, cspec)
        assert_allclose(first_waypoint, self.waypoint1)
        assert_allclose(last_waypoint, self.waypoint3)


class RetimeTrajectoryTest(object):
    def setUp(self):
        cspec = self.robot.GetActiveConfigurationSpecification('linear')

        self.feasible_path = openravepy.RaveCreateTrajectory(self.env, '')
        self.feasible_path.Init(cspec)
        self.feasible_path.Insert(0, self.waypoint1)
        self.feasible_path.Insert(1, self.waypoint2)
        self.feasible_path.Insert(2, self.waypoint3)

        self.dt = 0.01
        self.tolerance = 0.1  # 10% error

    def test_RetimeTrajectory(self):
        # Setup/Test
        traj = self.planner.RetimeTrajectory(self.robot, self.feasible_path)

        # Assert
        position_cspec = self.feasible_path.GetConfigurationSpecification()
        velocity_cspec = position_cspec.ConvertToDerivativeSpecification(1)
        zero_dof_values = numpy.zeros(position_cspec.GetDOF())

        # Verify that the trajectory passes through the original waypoints.
        waypoints = [self.waypoint1, self.waypoint2, self.waypoint3]
        waypoint_indices = [None] * len(waypoints)

        for iwaypoint in xrange(traj.GetNumWaypoints()):
            joint_values = traj.GetWaypoint(iwaypoint, position_cspec)

            # Compare the waypoint against every input waypoint.
            for icandidate, candidate_waypoint in enumerate(waypoints):
                if numpy.allclose(joint_values, candidate_waypoint):
                    self.assertIsNone(waypoint_indices[icandidate])
                    waypoint_indices[icandidate] = iwaypoint

        self.assertEquals(waypoint_indices[0], 0)
        self.assertEquals(waypoint_indices[-1], traj.GetNumWaypoints() - 1)

        for iwaypoint in waypoint_indices:
            self.assertIsNotNone(iwaypoint)

            # Verify that the velocity at the waypoint is zero.
            joint_velocities = traj.GetWaypoint(iwaypoint, velocity_cspec)
            assert_allclose(joint_velocities, zero_dof_values)

        # Verify the trajectory between waypoints.
        for t in numpy.arange(self.dt, traj.GetDuration(), self.dt):
            iafter = traj.GetFirstWaypointIndexAfterTime(t)
            ibefore = iafter - 1

            joint_values = traj.Sample(t, position_cspec)
            joint_values_before = traj.GetWaypoint(ibefore, position_cspec)
            joint_values_after = traj.GetWaypoint(iafter, position_cspec)

            distance_full = numpy.linalg.norm(
                joint_values_after - joint_values_before)
            distance_before = numpy.linalg.norm(
                joint_values - joint_values_before)
            distance_after = numpy.linalg.norm(
                joint_values - joint_values_after)
            deviation = distance_before + distance_after - distance_full
            self.assertLess(deviation, self.tolerance * distance_full)

        # Check joint limits and dynamic feasibility.
        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)

        openravepy.planningutils.VerifyTrajectory(params, traj, self.dt)

    # TODO: Test failure cases.


# Planner-specific tests
#
# Each of these classes MUST EXTEND BasePlannerTest, one or more
# method-specific test classes (e.g. PlanToConfigurationTests), and the
# unittest.TestCase class. The unittest.TestCase class MUST APPEAR LAST in the
# list of base classes.

class CBiRRTPlannerTests(BasePlannerTest,
                         PlanToConfigurationTest,
                         PlanToEndEffectorPoseTest,
                         unittest.TestCase):
    planner_factory = CBiRRTPlanner


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       unittest.TestCase):
    planner_factory = OMPLPlanner


class OMPLSimplifierTests(BasePlannerTest,
                          ShortcutPathTest,
                          unittest.TestCase):
    planner_factory = OMPLSimplifier

    def setUp(self):
        BasePlannerTest.setUp(self)
        ShortcutPathTest.setUp(self)


class ParabolicRetimerTests(BasePlannerTest,
                            RetimeTrajectoryTest,
                            unittest.TestCase):
    planner_factory = ParabolicRetimer

    def setUp(self):
        BasePlannerTest.setUp(self)
        RetimeTrajectoryTest.setUp(self)


if __name__ == '__main__':
    unittest.main()
