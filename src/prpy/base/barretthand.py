import numpy, openravepy
from .. import util
from endeffector import EndEffector

class BarrettHand(EndEffector):
    def __init__(self, sim, manipulator, owd_namespace, bhd_namespace, ft_sim=True):
        EndEffector.__init__(self, manipulator)

        self.simulated = sim

        # Hand controller
        robot = self.manipulator.GetRobot()
        env = robot.GetEnv()
        self.controller = robot.AttachController(name=self.GetName(),
            args='BHController {0:s} {1:s}'.format('prpy', bhd_namespace),
            dof_indices=self.GetIndices(), affine_dofs=0, simulated=sim)

        # Hand state, force/torque sensor, and tactile pads.
        if not sim:
            self.bh_sensor = util.create_sensor(env, 'HandstateSensor {0:s} {1:s}'.format('prpy', bhd_namespace))

        if not ft_sim:
            self.ft_sensor = util.create_sensor(env, 'BarrettFTSensor {0:s} {1:s}'.format('prpy', owd_namespace))

        # TODO: Attach the tactile sensor plugin.

    def MoveHand(hand, f1=None, f2=None, f3=None, spread=None, timeout=None):
        """
        Change the hand preshape. This function blocks until trajectory execution
        finishes. This can be changed by changing the timeout parameter to a
        maximum number of seconds. Pass zero to return instantantly.
        @param f1 finger 1 angle
        @param f2 finger 2 angle
        @param f3 finger 3 angle
        @param spread spread angle
        @param timeout blocking execution timeout
        """
        # Default any None's to the current DOF values.
        preshape = hand.GetDOFValues()
        if f1     is not None: preshape[0] = f1
        if f2     is not None: preshape[1] = f2
        if f3     is not None: preshape[2] = f3
        if spread is not None: preshape[3] = spread

        hand.controller.SetDesired(preshape)
        util.WaitForControllers([ hand.controller ], timeout=timeout) 
       
    def OpenHand(hand, spread=None, timeout=None):
        """
        Open the hand with a fixed spread.
        @param spread hand spread
        @param timeout blocking execution timeout
        """
        if hand.simulated:
            robot = hand.manipulator.GetRobot()
            p = openravepy.KinBody.SaveParameters

            with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
                hand.manipulator.SetActive()
                robot.task_manipulation.ReleaseFingers()

            util.WaitForControllers([ hand.controller ], timeout=timeout)
        else:
            # TODO: Load this angle from somewhere.
            hand.MoveHand(f1=0.0, f2=0.0, f3=0.0, spread=spread, timeout=timeout)

    def CloseHand(hand, spread=None, timeout=None):
        """
        Close the hand with a fixed spread.
        @param spread hand spread
        @param timeout blocking execution timeout
        """
        if hand.simulated:
            robot = hand.manipulator.GetRobot()
            p = openravepy.KinBody.SaveParameters

            with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
                hand.manipulator.SetActive()
                robot.task_manipulation.CloseFingers()

            util.WaitForControllers([ hand.controller ], timeout=timeout)
        else:
            # TODO: Load this angle from somewhere.
            hand.MoveHand(f1=3.2, f2=3.2, f3=3.2, spread=spread, timeout=timeout)

    def GetStrain(hand):
        """
        Gets the most recent strain sensor readings.
        @return a list of strain for each finger
        """
        if not hand.simulated:
            # This is because we are overriding the force/torque sensor datatype
            sensor_data = hand.handstate_sensor.GetSensorData()
            return sensor_data.force.copy()
        else:
            return numpy.zeros(3)

    def GetBreakaway(hand):
        """
        Gets the most recent breakaway readings for each finger
        @return a list of breakaway flags for each finger
        """
        if not hand.simulated:
            # This is because we are overriding the force/torque sensor datatype.
            sensor_data = hand.handstate_sensor.GetSensorData()
            breakaway = sensor_data.torque 
            return breakaway
        else:
            return [ False, False, False ]

    def GetForceTorque(hand):
        """
        Gets the most recent force/torque sensor reading in the hand frame.
        @return force,torque force/torque in the hand frame
        """
        if not hand.ft_simulated:
            sensor_data = hand.ft_sensor.GetSensorData()
            return sensor_data.force, sensor_data.torque
        else:
            return numpy.zeros(3), numpy.zeros(3)

    def TareForceTorqueSensor(hand):
        """
        Tare the force/torque sensor. This is necessary before using the sensor
        whenever the arm configuration has changed.
        """
        if not hand.ft_simulated:
            hand.ft_sensor.SendCommand('Tare')
            time.sleep(2)
