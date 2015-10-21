import numpy
from prpy.clone import CloneException
from prpy.planning.base import PlanningError
from prpy.planning.exceptions import (
    CollisionPlanningError,
    SelfCollisionPlanningError,
    JointLimitError)


class PlanToEndEffectorOffsetTest(object):
    # This configuration is feasible, but it's not possible to move up due to
    # collision with the environment.
    config_infeasible_env_movement = numpy.array([
        2.4393138295135239, 1.92465643247649250, -0.74016692563109798,
        1.5874962800295755, 0.81768598344294929, -1.2573553146693555,
        2.35727649844962
    ])

    # This configuration is feasible, but it's not possible to move down due to
    # self-collision.
    config_infeasible_self_movement = numpy.array([
        -0.0057795025512650264, -1.14744762192955350,  2.5500000000000003,
         2.5469121802527628000, -0.88347846589538992, -1.73131950448453797,
        -2.6602830007072269
    ])

    def setUp(self):
        self.direction = numpy.array([ 0., 0., 1. ])
        self.distance = 0.1

    def test_PlanToEndEffectorOffset_SatisfiesConstraint(self):
        from numpy.testing import assert_allclose

        # Setup
        with self.env:
            cspec = self.robot.GetActiveConfigurationSpecification()
            self.robot.SetActiveDOFValues(self.config_feasible_start)
            start_ik = self.manipulator.GetEndEffectorTransform()

        # Test
        path = self.planner.PlanToEndEffectorOffset(
            self.robot, direction=self.direction, distance=self.distance)

        # Assert
        self.ValidatePath(path)

        distance = 0.
        first_waypoint = path.GetWaypoint(0)
        assert_allclose(first_waypoint, self.config_feasible_start)

        for iwaypoint in xrange(1, path.GetNumWaypoints()):
            with self.env:
                waypoint = path.GetWaypoint(iwaypoint, cspec)
                self.robot.SetActiveDOFValues(waypoint)
                actual_ik = self.manipulator.GetEndEffectorTransform()

            # Verify that the position is monotone.
            distance = numpy.linalg.norm(actual_ik[0:3, 3] - start_ik[0:3, 3])
            expected_ik = start_ik.copy()
            expected_ik[0:3, 3] += distance * numpy.array([ 0., 0., 1. ])

            self.assertTransformClose(actual_ik, expected_ik,
                linear_tol=0.005, angular_tol=0.2)

        self.assertAlmostEqual(distance, 0.1, delta=0.005)

    def test_PlanToEndEffectorOffset_StartInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorOffset(
                self.robot, direction=self.direction, distance=self.distance)

    def test_PlanToEndEffectorOffset_StartInSelfCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_self_collision)

        # Test/Assert
        with self.assertRaises((PlanningError, CloneException)):
            self.planner.PlanToEndEffectorOffset(
                self.robot, direction=self.direction, distance=self.distance)

    def test_PlanToEndEffectorOffset_GoalInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_infeasible_env_movement)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorOffset(
                self.robot, direction=self.direction, distance=self.distance)

    def test_PlanToEndEffectorOffset_GoalInCollision_Throws(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_infeasible_self_movement)

        # Test/Assert
        with self.assertRaises(PlanningError):
            self.planner.PlanToEndEffectorOffset(
                self.robot, direction=numpy.array([0., 0., -1.]),
                distance=0.1)

class PlanToEndEffectorOffsetCollisionTest(object):
    """ Expects collision-specific error"""

    def test_PlanToEndEffectorOffset_StartInCollision_Throws_CollisionError(self):
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.config_env_collision)

        # Test/Assert
        with self.assertRaises((CollisionPlanningError,
            SelfCollisionPlanningError,
            JointLimitError)):
            self.planner.PlanToEndEffectorOffset(
                self.robot, direction=self.direction, distance=self.distance)