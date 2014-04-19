import roslib; roslib.load_manifest('prpy')
import openravepy, unittest, numpy, threading
import prpy.planning
from planner_mocks import SuccessPlanner, FailPlanner

class MetaPlannerTests(unittest.TestCase):
    def setUp(self):
        self.join_timeout = 5.0
        self.env = openravepy.Environment()
        self.env.Load('data/lab1.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')

        # Create a valid trajectory used to test planner successes.
        cspec = self.robot.GetActiveConfigurationSpecification()
        self.traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')
        self.traj.Init(cspec)
        self.traj.Insert(0, numpy.zeros(cspec.GetDOF()))
        self.traj.Insert(1, numpy.ones(cspec.GetDOF()))

class SequenceTests(MetaPlannerTests):
    def test_FirstPlannerSucceeds_SecondPlannerIsNotCalled(self):
        first_planner = SuccessPlanner(self.traj)
        second_planner = SuccessPlanner(self.traj)

        planner = prpy.planning.Sequence(first_planner, second_planner)
        planner.PlanTest(self.robot)

        self.assertEqual(first_planner.num_calls, 1)
        self.assertEqual(second_planner.num_calls, 0)

    def test_FirstPlannerFails_SecondPlannerIsCalled(self):
        first_planner = FailPlanner()
        second_planner = SuccessPlanner(self.traj)

        planner = prpy.planning.Sequence(first_planner, second_planner)
        planner.PlanTest(self.robot)

        self.assertEqual(first_planner.num_calls, 1)
        self.assertEqual(second_planner.num_calls, 1)

    def test_AllPlannersFails_ThrowsPlanningError(self):
        planner = prpy.planning.Sequence(FailPlanner(), FailPlanner())
        with self.assertRaises(prpy.planning.PlanningError):
            planner.PlanTest()

class RankedTests(MetaPlannerTests):
    def test_FirstPlannerSucceeds_ReturnsImmediately(self):
        first_planner = SuccessPlanner(self.traj, delay=True)
        second_planner = FailPlanner()
        planner = prpy.planning.Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = threading.Thread(target=test_planner)
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
        planner = prpy.planning.Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = threading.Thread(target=test_planner)
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

    def test_SecondPlannerSucceeds_WaitfsForFirstPlanneer(self):
        first_planner = SuccessPlanner(self.traj, delay=True)
        second_planner = SuccessPlanner(self.traj, delay=True)
        planner = prpy.planning.Ranked(first_planner, second_planner)

        # Run the metaplanner in a separate thread so we have control over when
        # each delegate planner terminates.
        def test_planner():
            planner.PlanTest(self.robot)

        test_thread = threading.Thread(target=test_planner)
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
        planner = prpy.planning.Ranked(FailPlanner(), FailPlanner())
        with self.assertRaises(prpy.planning.PlanningError):
            planner.PlanTest(self.robot)

if __name__ == '__main__':
    unittest.main()
