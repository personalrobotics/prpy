from unittest import TestCase
from planning_helpers import FailPlanner, MetaPlannerTests, SuccessPlanner
from prpy.planning.base import Sequence


class SequenceTests(MetaPlannerTests,
                    TestCase):
    def test_FirstPlannerSucceeds_SecondPlannerIsNotCalled(self):
        first_planner = SuccessPlanner(self.traj)
        second_planner = SuccessPlanner(self.traj)

        planner = Sequence(first_planner, second_planner)
        planner.PlanTest(self.robot)

        self.assertEqual(first_planner.num_calls, 1)
        self.assertEqual(second_planner.num_calls, 0)

    def test_FirstPlannerFails_SecondPlannerIsCalled(self):
        first_planner = FailPlanner()
        second_planner = SuccessPlanner(self.traj)

        planner = Sequence(first_planner, second_planner)
        planner.PlanTest(self.robot)

        self.assertEqual(first_planner.num_calls, 1)
        self.assertEqual(second_planner.num_calls, 1)

    def test_AllPlannersFails_ThrowsPlanningError(self):
        from prpy.planning.base import PlanningError

        planner = Sequence(FailPlanner(), FailPlanner())

        with self.assertRaises(PlanningError):
            planner.PlanTest(self.robot)
