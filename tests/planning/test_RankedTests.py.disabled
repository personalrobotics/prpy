from unittest import TestCase
from threading import Thread
from planning_helpers import FailPlanner, MetaPlannerTests, SuccessPlanner
from prpy.planning.base import Ranked


class RankedTests(MetaPlannerTests,
                  TestCase):
    def test_FirstPlannerSucceeds_ReturnsImmediately(self):
        first_planner = SuccessPlanner(self.traj, delay=True)
        second_planner = FailPlanner()
        planner = Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = Thread(target=test_planner)
        test_thread.start()
        first_planner.wait_for_start()
        second_planner.wait_for_start()

        # Force the first planner to return success. The meta-planner should
        # immediately return success and the second planner should still be
        # running in the background.
        first_planner.finish()
        test_thread.join(timeout=self.join_timeout)
        self.assertFalse(test_thread.isAlive())

        # Clean up by terminating the second planner.
        second_planner.finish()
        test_thread.join()

    def test_FirstPlannerFails_ReturnsResultOfSecondPlanner(self):
        first_planner = FailPlanner(delay=True)
        second_planner = SuccessPlanner(self.traj, delay=True)
        planner = Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = Thread(target=test_planner)
        test_thread.start()
        first_planner.wait_for_start()
        second_planner.wait_for_start()

        # Force the first planner to return failure. The meta-planner should
        # wait for the second planner before declaring failure.
        first_planner.finish()

        # Force the second planner to return. This trajectory should be
        # immediately returned by the meta-planner.
        second_planner.finish()
        test_thread.join(timeout=self.join_timeout)
        self.assertFalse(test_thread.isAlive())

        # Clean up.
        test_thread.join()

    def test_SecondPlannerSucceeds_WaitsForFirstPlanner(self):
        first_planner = SuccessPlanner(self.traj, delay=True)
        second_planner = SuccessPlanner(self.traj, delay=True)
        planner = Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = Thread(target=test_planner)
        test_thread.start()
        first_planner.wait_for_start()
        second_planner.wait_for_start()

        # Force the second planner to return success. The metaplanner should
        # continue waiting for the first planner.
        second_planner.finish()
        test_thread.join(timeout=self.join_timeout)
        self.assertTrue(test_thread.isAlive())

        # Terminate the first planner. The result should be instantly returned.
        # TODO: Also check that the correct trajectory was returned.
        first_planner.finish()
        test_thread.join(timeout=self.join_timeout)
        self.assertFalse(test_thread.isAlive())
        
    def test_AllPlannersFail_ThrowsPlanningError(self):
        planner = Ranked(FailPlanner(), FailPlanner())
        with self.assertRaises(prpy.planning.PlanningError):
            planner.PlanTest(self.robot)
