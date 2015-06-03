from prpy.clone import CloneException
from prpy.planning.base import PlanningError

class PlanToEndEffectorPoseTest(object):
    def test_PlanToEndEffectorPose_GoalIsFeasible_FindsSolution(self):
        from numpy.testing import assert_allclose

        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_feasible_start)

        # Test
        path = self.planner.PlanToEndEffectorPose(self.robot, goal_ik)

        # Assert
        self.ValidatePath(path)

        first_waypoint = path.GetWaypoint(0)
        assert_allclose(first_waypoint, self.config_feasible_start)

        with self.env:
            last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)
            self.robot.SetActiveDOFValues(last_waypoint)
            last_ik = self.manipulator.GetEndEffectorTransform()

        self.assertTransformClose(last_ik, goal_ik)
        self.assertFalse(self.CollisionCheckPath(path))

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
