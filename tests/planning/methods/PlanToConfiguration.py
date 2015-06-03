from prpy.clone import CloneException
from prpy.planning.base import PlanningError

class PlanToConfigurationTest(object):
    def test_PlanToConfiguration_GoalIsFeasible_FindsSolution(self):
        from numpy.testing import assert_allclose

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

        self.ValidatePath(path)
        assert_allclose(first_waypoint, self.config_feasible_start)
        assert_allclose(last_waypoint, self.config_feasible_goal)
        self.assertFalse(self.CollisionCheckPath(path))

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
