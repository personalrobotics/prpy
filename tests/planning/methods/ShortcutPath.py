
class ShortcutPathTest(object):
    def setUp(self):
        from openravepy import RaveCreateTrajectory

        self.input_path = RaveCreateTrajectory(self.env, '')
        self.input_path.Init(
            self.robot.GetActiveConfigurationSpecification('linear'))
        self.input_path.Insert(0, self.waypoint1)
        self.input_path.Insert(1, self.waypoint2)
        self.input_path.Insert(2, self.waypoint3)

    def test_ShortcutPath_ShortcutExists_ReducesLength(self):
        from numpy.testing import assert_allclose

        # Setup/Test
        smoothed_path = self.planner.ShortcutPath(self.robot, self.input_path)

        # Assert
        self.assertEquals(smoothed_path.GetConfigurationSpecification(),
                          self.input_path.GetConfigurationSpecification())
        self.assertGreaterEqual(smoothed_path.GetNumWaypoints(), 2)

        n = smoothed_path.GetNumWaypoints()
        assert_allclose(smoothed_path.GetWaypoint(0),     self.waypoint1)
        assert_allclose(smoothed_path.GetWaypoint(n - 1), self.waypoint3)

        self.assertLess(self.ComputeArcLength(smoothed_path),
                        0.5 * self.ComputeArcLength(self.input_path))

    # TODO: Test some of the error cases.
