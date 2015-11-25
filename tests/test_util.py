from __future__ import print_function
import openravepy
import prpy.util
import unittest

import os # environ, path
import subprocess
import sys # stderr

import numpy # allclose, zeros


# Add the models included with OpenRAVE to the OPENRAVE_DATA path.
# These may not be available if the user manually set the OPENRAVE_DATA
# environmental variable, e.g. through openrave_catkin.
try:
    share_path = \
          subprocess.check_output(['openrave-config', '--share-dir']).strip()
    os.environ['OPENRAVE_DATA'] = os.path.join(share_path, 'data')
except subprocess.CalledProcessError as e:
    print('error: Failed using "openrave-config" to find the default'
          ' OPENRAVE_DATA path. Loading assets may fail.',
          file=sys.stderr)

# Initialize OpenRAVE.
openravepy.RaveInitialize(True)
openravepy.misc.InitOpenRAVELogging()
openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('wamtest1.env.xml')
        self.traj = openravepy.RaveCreateTrajectory(self.env, '')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')

        # Set all 7 DOF of the WAM arm to active
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator)
            self.active_dof_indices = self.robot.GetActiveDOFIndices()


    def CreateTrajectory(self, q_start, q_goal):
        """
        Create a trajectory between two joint configurations.
        (a straight line in joint space)
        """
        env = self.env
        robot = self.robot
        dof_indices = self.active_dof_indices

        traj = openravepy.RaveCreateTrajectory(env, '')
        cspec = robot.GetActiveConfigurationSpecification('linear')

        # Add first waypoint
        start_waypoint = numpy.zeros(cspec.GetDOF())
        cspec.InsertJointValues(start_waypoint, q_start, robot, \
                                dof_indices, False)
        traj.Init(cspec)
        traj.Insert(0, start_waypoint.ravel())

        # Add second waypoint, if different to first
        if not numpy.allclose(q_start, q_goal):
            goal_waypoint = numpy.zeros(cspec.GetDOF())
            cspec.InsertJointValues(goal_waypoint, q_goal, robot, \
                                    dof_indices, False)
            traj.Insert(1, goal_waypoint.ravel())

        return traj


    # IsTimedTrajectory()

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


    # IsAtConfiguration()

    def test_IsAtConfiguration_ReturnsTrue(self):
        curr_config = self.robot.GetDOFValues(self.active_dof_indices)
        self.assertTrue(prpy.util.IsAtConfiguration(self.robot, curr_config))

    def test_IsAtConfiguration_ReturnsFalse(self):
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0];
        self.assertFalse(prpy.util.IsAtConfiguration(self.robot, goal_config))


    # IsAtTrajectoryStart()

    def test_IsAtTrajectoryStart_ReturnsTrue(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0];
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q0, q1)
        self.assertTrue(prpy.util.IsAtTrajectoryStart(self.robot, traj))

        # Test with only 1 waypoint
        traj = self.CreateTrajectory(q0, q0)
        self.assertTrue(prpy.util.IsAtTrajectoryStart(self.robot, traj))

    def test_IsAtTrajectoryStart_ReturnsFalse(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0];
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q1, q0) # goal is 1st waypoint
        self.assertFalse(prpy.util.IsAtTrajectoryStart(self.robot, traj))


    # IsAtTrajectoryEnd()

    def test_IsAtTrajectoryEnd_ReturnsTrue(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0];
        q0 = current_config
        q1 = goal_config
        # set last waypoint to current config
        traj = self.CreateTrajectory(q1, q0)
        self.assertTrue(prpy.util.IsAtTrajectoryEnd(self.robot, traj))

    def test_IsAtTrajectoryEnd_ReturnsFalse(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0];
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q0, q1)
        self.assertFalse(prpy.util.IsAtTrajectoryEnd(self.robot, traj))
    
    def test_ComputeUnitTiming(self):
        cspec = self.robot.GetActiveConfigurationSpecification()
        traj = openravepy.RaveCreateTrajectory(self.env, '')
        traj.Init(cspec)
        traj.Insert(0, [0,0,0,0,0,0,0])
        traj.Insert(1, [1,0,0,0,0,0,0])
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj,
            self.robot, False, 100.0, 100.0, 'LinearTrajectoryRetimer', '')
        traj_arclen = prpy.util.ComputeUnitTiming(self.robot, traj)
        dofvals = traj_arclen.Sample(0.99, cspec)
        self.assertAlmostEqual(dofvals[0], 0.99)

if __name__ == '__main__':
    unittest.main()
