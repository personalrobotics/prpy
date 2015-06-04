
class SmoothTrajectoryTest(object):
    def setUp(self):
        from openravepy import RaveCreateTrajectory

        cspec = self.robot.GetActiveConfigurationSpecification('linear')

        self.feasible_path = RaveCreateTrajectory(self.env, '')
        self.feasible_path.Init(cspec)
        self.feasible_path.Insert(0, self.waypoint1)
        self.feasible_path.Insert(1, self.waypoint2)
        self.feasible_path.Insert(2, self.waypoint3)

    def test_SmoothTrajectory_DoesNotModifyBoundaryPoints(self):
        from numpy.testing import assert_allclose

        # Setup/Test
        traj = self.planner.RetimeTrajectory(self.robot, self.feasible_path)

        # Assert
        cspec = self.robot.GetActiveConfigurationSpecification('linear')
        self.assertGreaterEqual(traj.GetNumWaypoints(), 2)

        first_waypoint = traj.GetWaypoint(0, cspec)
        last_waypoint = traj.GetWaypoint(traj.GetNumWaypoints() - 1, cspec)
        assert_allclose(first_waypoint, self.waypoint1)
        assert_allclose(last_waypoint, self.waypoint3)

