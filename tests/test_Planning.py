#!/usr/bin/env python
import openravepy, unittest, numpy, numpy.testing
from openravepy import (Environment, IkFilterOptions, IkParameterization,
                        IkParameterizationType)
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.ompl import OMPLPlanner


# Generic test setup
#
# This class is the base class of all planner tests. It is responsible for
# setting up the environment, but does directly not implement any tests.
class BasePlannerTest(object):
    """Generic environment setup.
    """
    active_dof_indices = range(7)
    start_config = numpy.array([
         2.35061574,  0.61043555,  0.85      ,  1.80684444, -0.08639935,
        -0.69750474,  1.31656172
    ])
    goal_config = numpy.array([
        -0.84085883,  1.44573701,  0.2       ,  1.72620231, -0.81124757,
        -1.39363597,  1.29233111
    ])

    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('data/wamtest2.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')

        # TODO: Planning should succeed with the floor present if CO_ActiveOnly
        # is set.
        self.env.Remove(self.env.GetKinBody('floor'))

        with self.env:
            self.robot.SetActiveManipulator(self.manipulator)
            self.robot.SetActiveDOFs(self.active_dof_indices)

            self.robot.SetActiveDOFValues(self.goal_config)
            self.goal_ik = self.manipulator.GetEndEffectorTransform()

            self.robot.SetActiveDOFValues(self.start_config)

        self.planner = self.planner_factory()

    def tearDown(self):
        self.env.Destroy()

    def _CollisionCheckPath(self, traj):
        # NOTE: This assumes that the trajectory only contains joint_values.
        OpenStart = openravepy.Interval.OpenStart

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(self.robot)

        # Check the first waypoint for collision. We do this outside of the
        # loop so we can run all collision checks with OpenEnd. This prevents
        # us from collision checking the intermediate waypoints twice.
        prev_waypoint = traj.GetWaypoint(0)
        check = params.CheckPathAllConstraints(prev_waypoint, prev_waypoint,
                                               [], [], 0., OpenStart)
        if check != 0:
            return True

        # Check the remainder of the path.
        for iwaypoint in xrange(1, traj.GetNumWaypoints() - 1):
            curr_waypoint = traj.GetWaypoint(iwaypoint)

            check = params.CheckPathAllConstraints(prev_waypoint, curr_waypoint,
                                                   [], [], 0., OpenStart)
            if check != 0:
                return True

            prev_waypoint = curr_waypoint

        return False


# Method-specific tests
#
# Methods from BasePlannerTest are also available in these classes. However,
# they should NOT inherit from BasePlannerTest.
class PlanToConfigurationTests(object):
    def test_PlanToConfiguration_FindsSolution(self):
        with self.env:
            path = self.planner.PlanToConfiguration(self.robot, self.goal_config)

        # pathectory should be in the input environment, not the cloned
        # planning environment.
        self.assertEquals(path.GetEnv(), self.env)

        # Output must be a geometric path. This means that it should not have
        # timing information, velocities, or accelerations.
        self.assertEquals(self.robot.GetActiveConfigurationSpecification(),
                          path.GetConfigurationSpecification())

        # Output must start in the start configuration and end in the goal
        # configuration.
        self.assertGreaterEqual(path.GetNumWaypoints(), 1)
        first_waypoint = path.GetWaypoint(0)
        last_waypoint = path.GetWaypoint(path.GetNumWaypoints() - 1)

        numpy.testing.assert_allclose(first_waypoint, self.start_config)
        numpy.testing.assert_allclose(last_waypoint, self.goal_config)

        # Path must be collision-free.
        self.assertFalse(self._CollisionCheckPath(path))


# Planner-specific tests
#
# Each of these classes MUST EXTEND BasePlannerTest, one or more
# method-specific test classes (e.g. PlanToConfigurationTests), and the
# unittest.TestCase class. The unittest.TestCase class MUST APPEAR LAST in the
# list of base classes.
"""
class OMPLPlannerTests(BasePlannerTest, PlanToConfigurationTests,
                      unittest.TestCase): 
    planner_factory = OMPLPlanner
"""

class CBiRRTPlannerTests(BasePlannerTest, PlanToConfigurationTests,
                         unittest.TestCase): 
    planner_factory = CBiRRTPlanner

if __name__ == '__main__':
    openravepy.RaveInitialize(True)

    import openravepy
    openravepy.misc.InitOpenRAVELogging()
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Verbose)

    unittest.main()
