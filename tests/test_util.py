import openravepy
import prpy.util
import unittest

class IsTimedTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.env = openravepy.Environment()
        self.traj = openravepy.RaveCreateTrajectory(self.env, '')

    def test_IsTimed_ReturnsTrue(self):
        cspec = openravepy.ConfigurationSpecification()
        cspec.AddDeltaTimeGroup()

        self.traj.Init(cspec)
        self.assertTrue(prpy.util.IsTimedTrajectory(self.traj))

        self.traj.Insert(0, [0.])
        self.assertTrue(prpy.util.IsTimedTrajectory(self.traj))

    def test_IsNotTimed_ReturnsFalse(self):
        cspec = openravepy.ConfigurationSpecification()
        cspec.AddGroup('joint_values test_robot 0', 1, 'linear')

        self.traj.Init(cspec)
        self.assertFalse(prpy.util.IsTimedTrajectory(self.traj))

        self.traj.Insert(0, [0.])
        self.assertFalse(prpy.util.IsTimedTrajectory(self.traj))
