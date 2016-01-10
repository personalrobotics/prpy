from prpy.clone import CloneException
from prpy.planning.base import PlanningError
from prpy.planning.exceptions import (
    CollisionPlanningError,
    SelfCollisionPlanningError,
    JointLimitError)

class PlanToConfigurationTest(object):
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

    def PlanFromStartToGoalConfiguration(self, config_start, config_goal):
        from numpy.testing import assert_allclose

        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(config_start)

        # Test
        path = self.planner.PlanToConfiguration(self.robot, config_goal)

        # Assert.
        self.assertEquals(path.GetEnv(), self.env)
        self.assertEquals(
            self.robot.GetActiveConfigurationSpecification('linear'),
            path.GetConfigurationSpecification())
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)
        first_waypoint = path.GetWaypoint(0)
        last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)

        self.ValidatePath(path)
        assert_allclose(first_waypoint, config_start)
        assert_allclose(last_waypoint, config_goal)
        self.assertFalse(self.CollisionCheckPath(path))


class PlanToConfigurationStraightLineTest(object):
    def test_PlanToConfiguration_AlreadyAtGoal_FindsSolution(self):
        self.PlanFromStartToGoalConfiguration(self.waypoint1, self.waypoint1)
        self.PlanFromStartToGoalConfiguration(self.waypoint2, self.waypoint2)
        self.PlanFromStartToGoalConfiguration(self.waypoint3, self.waypoint3)

    def test_PlanToConfiguration_StraightLineIsFeasible_FindsSolution(self):
        self.PlanFromStartToGoalConfiguration(self.waypoint1, self.waypoint2)
        self.PlanFromStartToGoalConfiguration(self.waypoint2, self.waypoint3)
        self.PlanFromStartToGoalConfiguration(self.waypoint3, self.waypoint1)


class PlanToConfigurationCompleteTest(object):
    def test_PlanToConfiguration_GoalIsFeasible_FindsSolution(self):
        self.PlanFromStartToGoalConfiguration(
            self.config_feasible_start2, self.config_feasible_goal)


class PlanToConfigurationTestCollisionTest(object): 
    """ Expects collision-specific error"""

    def test_PlanToConfiguration_GoalInCollision_Throws_Collision(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test/Assert
        with self.assertRaises((CollisionPlanningError,
            SelfCollisionPlanningError,
            JointLimitError)):
                self.planner.PlanToConfiguration(
                self.robot, self.config_env_collision)
