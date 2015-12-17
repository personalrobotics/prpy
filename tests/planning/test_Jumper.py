import unittest
from prpy.planning.jumper import Jumper
from prpy.planning.base import PlanningError
import openravepy
import numpy
from numpy.testing import assert_allclose

class JumperTest(unittest.TestCase):
    def setUp(self): 
        self.active_dof_indices = range(7)

        openravepy.RaveInitialize(True)
        openravepy.misc.InitOpenRAVELogging()
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

        self.env = openravepy.Environment()
        with self.env:
            self.env.Load('data/wamtest2.env.xml')
            self.robot = self.env.GetRobot('BarrettWAM')
            self.manipulator = self.robot.GetManipulator('arm')

            self.env.Remove(self.env.GetKinBody('floor'))

            self.robot.SetActiveManipulator(self.manipulator)
            self.robot.SetActiveDOFs(self.active_dof_indices)

        # Feasible start/goal pair.
        self.config_feasible_start = numpy.array([
            +2.35061574,  0.61043555,  0.85000000,  1.80684444, -0.08639935,
            -0.69750474,  1.31656172
        ])

        # This configuration is easier for Trajopt (and harder for most other
        # planners) because teh pole introduces a local minimum (maybe?).
        self.config_feasible_start2 = numpy.array([
            -2.61799387,  0.66440338,  0.19853743,  2.00944286, -0.08639925,
            -0.69750467,  1.31656153
        ])


        self.config_feasible_goal = numpy.array([
            -0.84085883,  1.44573701,  0.20000000,  1.72620231, -0.81124757,
            -1.39363597,  1.29233111
        ])

        # This configuration is in collision with the environment, but is not in
        # self-collision.
        self.config_env_collision = numpy.array([
            +3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
            +2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
            -2.07807269e-01
        ])

        # This configuration is in self-collision, but is not in collision with the
        # environment.
        self.config_self_collision = numpy.array([
            2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
            2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
            0.00000000e+00
        ])
        
        self.direction = numpy.array([ 0., 0., 1. ])
        self.distance = 0.1

        self.jumper = Jumper()

    def test_JumpToConfigurationNoCollision(self):
        # Setup 
        with self.env:
            self.jumper.JumpToConfiguration(self.env, self.robot, 
                                            self.active_dof_indices,
                                            self.config_feasible_goal)
            end_config = self.robot.GetActiveDOFValues()

        # Test
        assert_allclose(self.config_feasible_goal, end_config)

    def test_JumpToConfigurationInCollision(self): 
        # Test
        with self.assertRaises(PlanningError):
            with self.env: 
                self.jumper.JumpToConfiguration(self.env, self.robot,
                                                self.active_dof_indices,
                                                self.config_env_collision)

    def test_JumpToEndEffectorOffset(self): 
        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)
            start_pose = self.manipulator.GetEndEffectorTransform()

            # Test
            self.jumper.JumpToEndEffectorOffset(self.env, self.robot, 
                                                self.active_dof_indices, 
                                                self.direction, 
                                                self.distance)

            end_pose = self.manipulator.GetEndEffectorTransform()

            # Verify that the position is monotone.
            expected_pose = start_pose.copy()
            expected_pose[0:3, 3] += self.distance * self.direction

            assert_allclose(expected_pose, end_pose)


    def test_JumpToEndEffectorPoseNoCollision(self):

        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_pose = self.manipulator.GetEndEffectorTransform()

        with self.env: 
            self.jumper.JumpToEndEffectorPose(self.env, self.robot, 
                                                self.active_dof_indices, 
                                                goal_pose)
            last_pose = self.manipulator.GetEndEffectorTransform()

        # Test
        assert_allclose(goal_pose, last_pose)


    def test_JumpToEndEffectorPoseInCollision(self): 
        # Setup 
        with self.env: 
            self.robot.SetActiveDOFValues(self.config_env_collision)
            goal_pose = self.manipulator.GetEndEffectorTransform()

        # Test
        with self.assertRaises(PlanningError):
            with self.env: 
                self.jumper.JumpToEndEffectorPose(self.env, self.robot, 
                                                    self.active_dof_indices,
                                                    goal_pose)


if __name__ == '__main__':
    unittest.main()