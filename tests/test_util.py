from __future__ import print_function
import openravepy
import prpy.util
import unittest

import os # environ, path
import subprocess
import sys # stderr

import numpy # allclose, zeros
import numpy.testing # assert_array_almost_equal
import exceptions # Exception
import itertools # islice
from prpy.planning.exceptions import JointLimitError


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


class Tests(unittest.TestCase):
    """
    Various unit tests.
    """
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

        # Get the resolution (in radians) for the 7 joints
        # [0.0043, 0.0087, 0.0087, 0.0174, 0.0193, 0.0282, 0.0282]
        self.dof_resolutions = \
               self.robot.GetDOFResolutions()[self.active_dof_indices]


    def tearDown(self):
        self.env.Destroy()


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
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0]
        self.assertFalse(prpy.util.IsAtConfiguration(self.robot, goal_config))


    # IsAtTrajectoryStart()

    def test_IsAtTrajectoryStart_ReturnsTrue(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0]
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q0, q1)
        self.assertTrue(prpy.util.IsAtTrajectoryStart(self.robot, traj))

        # Test with only 1 waypoint
        traj = self.CreateTrajectory(q0, q0)
        self.assertTrue(prpy.util.IsAtTrajectoryStart(self.robot, traj))

    def test_IsAtTrajectoryStart_ReturnsFalse(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0]
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q1, q0) # goal is 1st waypoint
        self.assertFalse(prpy.util.IsAtTrajectoryStart(self.robot, traj))


    # IsAtTrajectoryEnd()

    def test_IsAtTrajectoryEnd_ReturnsTrue(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0]
        q0 = current_config
        q1 = goal_config
        # set last waypoint to current config
        traj = self.CreateTrajectory(q1, q0)
        self.assertTrue(prpy.util.IsAtTrajectoryEnd(self.robot, traj))

    def test_IsAtTrajectoryEnd_ReturnsFalse(self):
        current_config = self.robot.GetDOFValues(self.active_dof_indices)
        goal_config = [0.02, 0.01, 0.02, 0.01, 0.01, 0.01, 0.0]
        q0 = current_config
        q1 = goal_config
        traj = self.CreateTrajectory(q0, q1)
        self.assertFalse(prpy.util.IsAtTrajectoryEnd(self.robot, traj))


    # ComputeUnitTiming()
    
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


    # CheckJointLimits()
    #
    # Note: the WAM arm joint limits are:
    #       q_limit_min =
    #           [-2.617, -1.972, -2.740, -0.872, -4.799, -1.570, -3.001]
    #       q_limit_max =
    #           [ 2.617,  1.972,  2.740,  3.141,  1.308,  1.570,  3.001]

    def test_CheckJointLimits_MinLimitOneJoint(self):
        # Individually check for when one joint is beyond the min limit
        for i in xrange(0,7):
            q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            q[i] = -5.0
            with self.assertRaises(JointLimitError):
                prpy.util.CheckJointLimits(self.robot, q)

    def test_CheckJointLimits_MinLimitAll(self):
        # This configuration is beyond the min limits
        q0 = [-2.7, -2.0, -2.8, -0.9, -4.9, -1.6, -3.1]
        with self.assertRaises(JointLimitError):
            prpy.util.CheckJointLimits(self.robot, q0)

    def test_CheckJointLimits_MaxLimitOneJoint(self):
        # Individually check for when one joint is beyond the max limit
        for i in xrange(0,7):
            q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            q[i] = +4.5
            with self.assertRaises(JointLimitError):
                prpy.util.CheckJointLimits(self.robot, q)

    def test_CheckJointLimits_MaxLimitAll(self):
        # This configuration is beyond the max limits
        q0 = [2.7, 2.0, 2.8, 0.9, 4.9, 1.6, 3.1]
        with self.assertRaises(JointLimitError):
            prpy.util.CheckJointLimits(self.robot, q0)

    def test_CheckJointLimits_WithinLimits_DoesNotThrow(self):
        # Check the zero position, which should succeed
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        try:
            prpy.util.CheckJointLimits(self.robot, q0)
        except JointLimitError:
            error = 'Unexpected exception thrown: ' + str(e.message)
            self.fail(error)
        except Exception, e:
            error = 'Unexpected exception thrown: ' + str(e.message)
            self.fail(error)
        else:
            pass # test passed


    # GetLinearCollisionCheckPts()

    def test_GetLinearCollisionCheckPts_SinglePointTraj(self):
        # Test a single point trajectory,
        # this should return 1 number of checks:
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        desired_num_checks = 1
        traj = self.CreateTrajectory(q0, q0) # makes traj with 1 waypoint
        # Linear sampling
        linear = prpy.util.SampleTimeGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=linear)
        num_checks = sum(1 for x in checks)
        if num_checks != desired_num_checks:
            error = str(num_checks) + ' is the wrong number of check pts.'
            self.fail(error)

    def test_GetLinearCollisionCheckPts_TwoPointTraj_LessThanDOFRes(self):
        # Test a two point trajectory that steps each joint
        # by half its DOF resolution.
        # This should return 2 number of checks:
        # 1. the start position
        # 2. the end position
        # because the L2 norm = 1.32
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q1 = 0.5 * self.dof_resolutions
        desired_num_checks = 2
        traj = self.CreateTrajectory(q0, q1)
        # Linear sampling
        linear = prpy.util.SampleTimeGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=linear)
        num_checks = 0
        for t, q in checks:
            num_checks = num_checks + 1
            prev_q = q # the robot configuration
        if num_checks != desired_num_checks:
            error = str(num_checks) + ' is the wrong number of check pts.'
            self.fail(error)

        # Also the last check point (configuration) should
        # be the same as the goal configuration.
        error = 'The last configuration to check ' + str(prev_q) + \
                ' should equal the goal configuration ' + str(q1)
        numpy.testing.assert_array_almost_equal(prev_q, q1, decimal=7, \
                                                err_msg=error, verbose=True)

    def test_GetLinearCollisionCheckPts_TwoPointTraj_EqualToDOFRes(self):
        # Test a two point trajectory that steps each joint
        # by its DOF resolution.
        # This should return 2 number of checks:
        # 1. the start position
        # 2. the position at t=2
        # Because the L2 norm = 2.64.
        # the actual end position does not need to be checked.
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q1 = 1.0 * self.dof_resolutions
        desired_num_checks = 2
        traj = self.CreateTrajectory(q0, q1)
        # Linear sampling
        linear = prpy.util.SampleTimeGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=linear)
        num_checks = 0
        for t, q in checks:
            num_checks = num_checks + 1
            prev_q = q # the robot configuration
        if num_checks != desired_num_checks:
            error = str(num_checks) + ' is the wrong number of check pts.'
            self.fail(error)

    def test_GetLinearCollisionCheckPts_TwoPointTraj_GreaterThanDOFRes(self):
        # Test a two point trajectory that steps each joint
        # by 1.2 times its DOF resolution.
        # This should return 3 number of checks:
        # 1. the start position
        # 2. the position at t=2
        # 3. the end position
        # because the L2 norm = 3.17.
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q1 = 1.2 * self.dof_resolutions
        desired_num_checks = 3
        traj = self.CreateTrajectory(q0, q1)
        # Linear sampling
        linear = prpy.util.SampleTimeGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=linear)
        num_checks = 0
        for t, q in checks:
            num_checks = num_checks + 1
            prev_q = q # the robot configuration
        if num_checks != desired_num_checks:
            error = str(num_checks) + ' is the wrong number of check pts.'
            self.fail(error)

        # Also the last check point (configuration) should be
        # the same as the goal configuration
        error = 'The last configuration to check ' + str(prev_q) + \
                ' should equal the goal configuration ' + str(q1)
        numpy.testing.assert_array_almost_equal(prev_q, q1, decimal=7, \
                                                err_msg=error, verbose=True)

    def test_GetLinearCollisionCheckPts_SamplingOrderMethods(self):
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q1 = [0.01, 0.02, 0.03, 0.04, 0.03, 0.02, 0.01]
        traj = self.CreateTrajectory(q0, q1)

        # Linear sampling
        linear = prpy.util.SampleTimeGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=linear)
        try:
            # Exception can be thrown only when we try to use the generator
            num_checks = sum(1 for x in checks)
        except ValueError:
            error = "Unknown sampling_func: '" + method + "' "
            self.fail(error)
        except Exception, e:
            error = 'Unexpected exception thrown: ' + str(e.message)
            self.fail(error)
        else:
            pass # test passed

        # An approximate Van der Corput sequence, between the
        # start and end points
        vdc = prpy.util.VanDerCorputSampleGenerator
        checks = prpy.util.GetLinearCollisionCheckPts(self.robot, \
                                                      traj, \
                                                      norm_order=2, \
                                                      sampling_func=vdc)
        try:
            # Exception can be thrown only when we try to use the generator
            num_checks = sum(1 for x in checks)
        except ValueError:
            error = "Unknown sampling_func: '" + method + "' "
            self.fail(error)
        except Exception, e:
            error = 'Unexpected exception thrown: ' + str(e.message)
            self.fail(error)
        else:
            pass # test passed


    # ConvertIntToBinaryString()

    def test_ConvertIntToBinaryString(self):
        # The integer 6 in binary is "110"
        binary_string = prpy.util.ConvertIntToBinaryString(6)
        expected_string = '110'
        error = 'The binary value ' + binary_string + \
                ' does not equal the expected value ' + expected_string
        self.assertEqual(binary_string, expected_string, msg=error)

        # The integer 6 in binary is "110"
        # so this call should return "011"
        binary_string = prpy.util.ConvertIntToBinaryString(6, reverse=True)
        expected_string = '011'
        error = 'The binary value ' + binary_string + \
                ' does not equal the expected value ' + expected_string
        self.assertEqual(binary_string, expected_string, msg=error)


    # VanDerCorputSequence()

    def test_VanDerCorputSequence_IncludeEndpoints(self):
        expected_sequence = [0.0, 1.0, 0.5, 0.25, 0.75]
        vdc = prpy.util.VanDerCorputSequence(lower=0.0, upper=1.0, \
                                             include_endpoints=True)
        num_values = 5
        vdc_seq = itertools.islice(vdc, num_values)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSequence_ExcludeEndpoints(self):
        expected_sequence = [0.5, 0.25, 0.75]
        vdc = prpy.util.VanDerCorputSequence(lower=0.0, upper=1.0, \
                                             include_endpoints=False)
        num_values = 3
        vdc_seq = itertools.islice(vdc, num_values)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSequence_RangeScaling(self):
        # this also tests using integers as input to the function
        expected_sequence = [0.0, 20.0, 10.0, 5.0, 15.0, 2.5, 12.5, 7.5, 17.5]
        start = 0
        end = 20
        vdc = prpy.util.VanDerCorputSequence(lower=start, upper=end, \
                                             include_endpoints=True)
        num_values = 9
        vdc_seq = itertools.islice(vdc, num_values)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSequence_PowerOfTwo(self):
        expected_sequence = [0.0, 8.0, 4.0, 2.0, 6.0, 1.0, 5.0, 3.0, 7.0]
        start = 0
        end = 2**3 # 8
        vdc = prpy.util.VanDerCorputSequence(lower=start, upper=end, \
                                             include_endpoints=True)
        num_values = end+1
        vdc_seq = itertools.islice(vdc, num_values)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)


    # VanDerCorputSampleGenerator()
    # (This function wraps VanDerCorputSequence() and is used
    #  for getting sample points to be collision-checked.)

    def test_VanDerCorputSampleGenerator_ExcessLessThanHalfStep(self):
        # Check that the end-point is included when it's
        # more than half the step-size from the closest value.
        expected_sequence = [0.0, 13.7, 12.0, 6.0, 4.0, 8.0, 2.0, 10.0]
        traj_dur = 13.7
        vdc_seq = prpy.util.VanDerCorputSampleGenerator(0.0, traj_dur, step=2)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSampleGenerator_ExcessEqualToHalfStep(self):
        # Check that the end-point is NOT included when it's distance 
        # is EQUAL too (or less than) the step-size.
        expected_sequence = [0.0, 12.0, 6.0, 4.0, 8.0, 2.0, 10.0]
        traj_dur = 13.0
        vdc_seq = prpy.util.VanDerCorputSampleGenerator(0.0, traj_dur, step=2)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSampleGenerator_NoExcess(self):
        # Check when the step-size fits exactly into the duration of the
        # trajectory, also check when function inputs are integers.
        expected_sequence = [0.0, 12.0, 6.0, 4.0, 8.0, 2.0, 10.0]
        traj_duratn = 12
        vdc_seq = prpy.util.VanDerCorputSampleGenerator(0, traj_duratn, step=2)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_VanDerCorputSampleGenerator_StepSizeSnapping(self):
        # Check that the sequence values are 'snapped' to the step-size
        # and also that all values from start to end are returned.
        #
        # That is, the 3rd value of the expected sequence is '6'
        # (instead of 5.5) because we need the Sample Generate to
        # snap all sequence values to the step-size.
        expected_sequence = [0.0, 11.0, 6.0, 3.0, 8.0, 1.0, \
                                                 7.0, 4.0, 10.0, 2.0, 5.0, 9.0]
        traj_duratn = 11
        vdc_seq = prpy.util.VanDerCorputSampleGenerator(0, traj_duratn, step=1)
        seq_list = [s for s in vdc_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)


    # SampleTimeGenerator()

    def test_SampleTimeGenerator_FloatStep(self):
        expected_sequence = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        stg = prpy.util.SampleTimeGenerator(start=0.0, end=1.0, step=0.2)
        num_values = 6
        stg_seq = itertools.islice(stg, num_values)
        seq_list = [s for s in stg_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_SampleTimeGenerator_IntStep(self):
        expected_sequence = [0, 1, 2, 3, 4, 5]
        stg = prpy.util.SampleTimeGenerator(start=0, end=5, step=1)
        num_values = 6
        stg_seq = itertools.islice(stg, num_values)
        seq_list = [s for s in stg_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_SampleTimeGenerator_StepOfTwo(self):
        # Check using step-size of 2, which is used when generating
        # a sequence of points for collision-checking.
        expected_sequence = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
        stg = prpy.util.SampleTimeGenerator(start=0, end=20, step=2)
        num_values = 11
        stg_seq = itertools.islice(stg, num_values)
        seq_list = [s for s in stg_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_SampleTimeGenerator_EndPointMoreThanHalfStepSize(self):
        # Check that the sequence includes the end-point when it's
        # distance from the previous point is more than half the
        # step-size. This is required for collision-checking
        # using an L2 norm.
        expected_sequence = [0.0, 2.0, 4.0, 5.7]
        stg = prpy.util.SampleTimeGenerator(start=0.0, end=5.7, step=2)
        num_values = 4
        stg_seq = itertools.islice(stg, num_values)
        seq_list = [s for s in stg_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)

    def test_SampleTimeGenerator_EndPointLessThanHalfStepSize(self):
        # Check that the end-point is excluded when it's less than
        # half the step-size from the previous value.
        expected_sequence = [0.0, 2.0, 4.0]
        stg = prpy.util.SampleTimeGenerator(start=0.0, end=4.9, step=2)
        num_values = 3
        stg_seq = itertools.islice(stg, num_values)
        seq_list = [s for s in stg_seq]
        error = 'The sequence ' + str(seq_list) + \
                ' doesnt match the expected sequence ' + str(expected_sequence)
        numpy.testing.assert_array_almost_equal(seq_list, expected_sequence, \
                                                decimal=7, err_msg=error, \
                                                verbose=True)


    # GetForwardKinematics()

    def test_GetForwardKinematics_UseActiveManipulator(self):
        # Zero configuration, uses the the active manipulator by default
        q0 = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        T_ee = prpy.util.GetForwardKinematics(self.robot, q0)

        expected_T_ee0 = numpy.array([[1.0, 0.0, 0.0, 0.07 ],
                                      [0.0, 1.0, 0.0, 0.0  ],
                                      [0.0, 0.0, 1.0, 2.168],
                                      [0.0, 0.0, 0.0, 1.0  ]])
        error = 'The end-effector transform ' + str(T_ee) + \
                ' does not equal the expected end-effector' + \
                ' transform ' + str(expected_T_ee0)
        numpy.testing.assert_array_almost_equal(T_ee, expected_T_ee0, decimal=7, \
                                                err_msg=error, verbose=True)

    def test_GetForwardKinematics_SpecifyManipulator(self):
        # A different configuration, specifying the manipulator to use
        q1 = numpy.array([-1.8, 0.0, -0.7, 2.0, 0.5, 0.2, 0.0])
        manip = self.robot.GetActiveManipulator()
        T_ee = prpy.util.GetForwardKinematics(self.robot, q1, manip)

        expected_T_ee1 = numpy.array([[ 0.7126777,  0.3653714, -0.5988272, -0.3313395],
                                      [-0.0541115, -0.8224716, -0.5662263, -0.3259651],
                                      [-0.6994014,  0.4359404, -0.5663864,  1.4394693],
                                      [ 0.0,        0.0,        0.0,        1.0      ]])
        error = 'The end-effector transform ' + str(T_ee) + \
                ' does not equal the expected end-effector' + \
                ' transform ' + str(expected_T_ee1)
        numpy.testing.assert_array_almost_equal(T_ee, expected_T_ee1, decimal=7, \
                                                err_msg=error, verbose=True)

    def test_GetForwardKinematics_SpecifyFrame(self):
        # Get the FK with respect to the base frame ('segway') of the
        # manipulator chain. This should give the same result as not
        # specifying the reference frame at all.
        q1 = numpy.array([-1.8, 0.0, -0.7, 2.0, 0.5, 0.2, 0.0])
        manip = self.robot.GetActiveManipulator()
        frame = 'segway'
        T_ee = prpy.util.GetForwardKinematics(self.robot, q1, manip, frame)

        expected_T_ee2 = numpy.array([[ 0.7126777,  0.3653714, -0.5988272, -0.3313395],
                                      [-0.0541115, -0.8224716, -0.5662263, -0.3259651],
                                      [-0.6994014,  0.4359404, -0.5663864,  1.4394693],
                                      [ 0.0,        0.0,        0.0,        1.0      ]])

        # We need to transform the expected result along the z-axis
        # because the Segway-WAM model is strange
        T_segway = self.robot.GetLink('segway').GetTransform()
        expected_T_ee2 = numpy.dot(numpy.linalg.inv(T_segway), expected_T_ee2)

        error = 'The end-effector transform ' + str(T_ee) + \
                ' does not equal the expected end-effector' + \
                ' transform ' + str(expected_T_ee2)
        numpy.testing.assert_array_almost_equal(T_ee, expected_T_ee2, decimal=7, \
                                                err_msg=error, verbose=True)


class Test_GetPointFrom(unittest.TestCase):
    """
    Unit Tests for GetPointFrom()
    """
    def setUp(self):
        self.env = openravepy.Environment()
        
    def tearDown(self):
        self.env.Destroy()

    def test_GetPointFrom_Kinbody(self):
        # Check that each input type returns the correct xyz coord

        # Kinbody
        self.env.Load('data/mug1.dae')
        mug = self.env.GetKinBody('mug')
        mug.SetTransform(numpy.eye(4)) 
        coord_kinbody = prpy.util.GetPointFrom(mug)
        expected_kinbody_coords = [0, 0, 0]
        numpy.testing.assert_array_almost_equal(coord_kinbody, expected_kinbody_coords)

    def test_GetPointFrom_Space(self):
        # Check that each input type returns the correct xyz coord
        expected_coord = [1, 3, 4]

        # Point in space
        space_coord = numpy.array([1, 3, 4])
        space_result = prpy.util.GetPointFrom(space_coord)
        numpy.testing.assert_array_almost_equal(space_result, expected_coord)

    def test_GetPointFrom_Transform(self):
        # Check that each input type returns the correct xyz coord
        expected_coord = [1, 3, 4]
 
        # 4x4 Transform
        trans_coord = numpy.eye(4)
        trans_coord[0:3, 3] = expected_coord
        trans_result = prpy.util.GetPointFrom(trans_coord)
        numpy.testing.assert_array_almost_equal(trans_result, expected_coord)

    def test_GetPointFrom_List(self):
        # Check that each input type returns the correct xyz coord
        expected_coord = [1, 3, 4]

        # List
        list_coord = [1, 3, 4]
        list_result = prpy.util.GetPointFrom(list_coord)
        numpy.testing.assert_array_almost_equal(list_result, expected_coord)


if __name__ == '__main__':
    unittest.main()
