import unittest
from prpy.planning.IKGenerator import IKGenerator
from prpy.planning.base import PlanningError
import openravepy
import numpy
from numpy.testing import assert_allclose

class IKGeneratorTest(unittest.TestCase):
    def setUp(self): 
        self.active_dof_indices = range(7)

        openravepy.RaveInitialize(True)
        openravepy.misc.InitOpenRAVELogging()
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

        self.env = openravepy.Environment()
        self.env.SetViewer('interactivemarker')

        # with self.env:
        if True:
            self.env.Load('data/wamtest2.env.xml')

        with self.env:

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

        self.ik_generator = IKGenerator(self.env, self.robot, self.manipulator)


    def test_FindIKSolutionsForConfigurationNoCollision(self):
        # Setup 
        with self.env:
            ik_solutions = self.ik_generator.FindIKSolutionsForConfiguration(
                                            self.active_dof_indices,
                                            self.config_feasible_goal)

            self.assertEquals(1, len(ik_solutions))
            assert_allclose(self.config_feasible_goal, ik_solutions[0])

    def test_FindIKSolutionsForConfigurationInCollision(self): 
        # Test
        with self.assertRaises(PlanningError):
            with self.env: 
                ik_solutions = self.ik_generator.FindIKSolutionsForConfiguration(
                                                self.active_dof_indices,
                                                self.config_env_collision)

    def test_FindIKSolutionsorEndEffectorOffset(self): 

        p = openravepy.KinBody.SaveParameters 

        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_start)
            start_pose = self.manipulator.GetEndEffectorTransform()

        ik_solutions = self.ik_generator.FindIKSolutionsForEndEffectorOffset(self.direction, 
                                                                        self.distance)

        expected_pose = start_pose.copy()
        expected_pose[0:3, 3] += self.distance * self.direction
        
        for q in ik_solutions: 
            with self.robot.CreateRobotStateSaver(p.LinkTransformation):
                with self.env: 
                    self.robot.SetActiveDOFValues(q)
                    actual_pose = self.manipulator.GetEndEffectorTransform()    
                assert_allclose(expected_pose, actual_pose)

    def test_FindIKSolutionsForEndEffectorPoseNoCollision(self):

        p = openravepy.KinBody.SaveParameters 

        # Setup
        with self.env:
            self.robot.SetActiveDOFValues(self.config_feasible_goal)
            goal_pose = self.manipulator.GetEndEffectorTransform()

        with self.env: 
            ik_solutions = self.ik_generator.FindIKSolutionsForEndEffectorPose(goal_pose)

        for q in ik_solutions: 
            with self.robot.CreateRobotStateSaver(p.LinkTransformation):
                with self.env: 
                    self.robot.SetActiveDOFValues(q)
                    actual_pose = self.manipulator.GetEndEffectorTransform()    
                assert_allclose(goal_pose, actual_pose)

    def test_FindIKSolutionsForEndEffectorPoseInCollision(self): 
        # Setup 
        with self.env: 
            self.robot.SetActiveDOFValues(self.config_env_collision)
            goal_pose = self.manipulator.GetEndEffectorTransform()

        # Test
        with self.assertRaises(PlanningError):
            with self.env: 
                self.ik_generator.FindIKSolutionsForEndEffectorPose(goal_pose)


if __name__ == '__main__':
    unittest.main()