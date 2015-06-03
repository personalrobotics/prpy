
class RetimeTrajectoryTest(object):
    def setUp(self):
        from openravepy import planningutils, Planner, RaveCreateTrajectory

        cspec = self.robot.GetActiveConfigurationSpecification('linear')

        self.feasible_path = RaveCreateTrajectory(self.env, '')
        self.feasible_path.Init(cspec)
        self.feasible_path.Insert(0, self.waypoint1)
        self.feasible_path.Insert(1, self.waypoint2)
        self.feasible_path.Insert(2, self.waypoint3)

        self.dt = 0.01
        self.tolerance = 0.1  # 10% error

    def test_RetimeTrajectory(self):
        import numpy
        from numpy.testing import assert_allclose
        from openravepy import planningutils, Planner

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
        params = Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)

        planningutils.VerifyTrajectory(params, traj, self.dt)

    # TODO: Test failure cases.
