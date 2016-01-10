#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy, openravepy, time
from .. import util
from endeffector import EndEffector

class BarrettHand(EndEffector):
    def __init__(self, sim, manipulator, owd_namespace, bhd_namespace, ft_sim=True):
        """End-effector wrapper for the BarrettHand.
        This class wraps a BarrettHand end-effector that is controlled by BHD
        or OWD. The or_owd_controller, or_handstate_sensor, and
        or_barrett_ft_sensor packages are used to communicate with the
        underlying hardware drivers through ROS. Note that some methods (e.g.
        reading breakaway status) are not supported on the BH-262.
        @param sim whether the hand is simulated
        @param manipulator manipulator the hand is attached to
        @param owd_namespace ROS namespace that OWD is running in
        @param bhd_namespace ROS namespace that the BarrettHand driver is running in
        @param ft_sim whether the force/torque sensor is simulated
        """
        EndEffector.__init__(self, manipulator)
        self.simulated = sim

        # Set the closing direction of the hand. This is only really necessary
        # for the programmatically loaded HERB model. We can't easily specify
        # ClosingDirection in URDF or SRDF.
        gripper_indices = manipulator.GetGripperIndices()
        closing_direction = numpy.zeros(len(gripper_indices))
        finger_indices = self.GetFingerIndices()

        for i, dof_index in enumerate(gripper_indices):
            if dof_index in finger_indices:
                closing_direction[i] = 1.

        manipulator.SetChuckingDirection(closing_direction)

        # Load the  hand controller.
        robot = self.manipulator.GetRobot()
        env = robot.GetEnv()
        self.controller = robot.AttachController(name=self.GetName(),
            args='BHController {0:s} {1:s}'.format('prpy', bhd_namespace),
            dof_indices=self.GetIndices(), affine_dofs=0, simulated=sim)

        # Hand state, force/torque sensor, and tactile pads.
        if not sim:
            self.handstate_sensor = util.create_sensor(env,
                'HandstateSensor {0:s} {1:s}'.format('prpy', bhd_namespace))

        self.ft_simulated = ft_sim
        if not ft_sim:
            self.ft_sensor = util.create_sensor(env,
                'BarrettFTSensor {0:s} {1:s}'.format('prpy', owd_namespace))

        # TODO: Attach the tactile sensor plugin.

    def GetSpreadIndex(self):
        """ Gets the DOF index of the spread joint.
        @return DOF index of the spread joint
        """
        return self._GetJointFromName('j00').GetDOFIndex()

    def GetFingerIndices(self):
        """ Gets the DOF indices of the fingers.
        These are returned in the order: [ finger 0, finger 1, and finger 2 ].
        @return DOF indices of the fingers
        """
        names = [ 'j01', 'j11', 'j21' ]
        return [ self._GetJointFromName(name).GetDOFIndex() for name in names ]

    def GetIndices(self):
        """ Gets the DOF indices of this hand.
        For compatability with OWD, the indices are always returned in the
        order: [ finger 0, finger 1, finger 2, spread ].
        @return DOF indices of the hand
        """
        return self.GetFingerIndices() + [ self.GetSpreadIndex() ]

    def MoveHand(hand, f1=None, f2=None, f3=None, spread=None, timeout=None):
        """Change the hand preshape.
        Joints that are not specified will not move. This function blocks until
        the hand has reached the desired configuration or a timeout occurs.
        Specifying a timeout of a finishes.  Specifying a timeout of None
        blocks forever and a timeout of zero returns instantly.  Note that the
        fingers are not collision-checked in simulation and may penetrate
        objects in the environment.
        @param f1 finger 1 angle, in radians
        @param f2 finger 2 angle, in radians
        @param f3 finger 3 angle, in radians
        @param spread spread angle, in radians
        @param timeout blocking execution timeout, in seconds
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
        """Open the hand with a fixed spread.
        This function blocks until the hand has reached the desired
        configuration or a timeout occurs. Specifying a timeout of a finishes.
        Specifying a timeout of None blocks forever and a timeout of zero
        returns instantly. Each finger is individually collision-checked in
        simulation and will stop if it collides with the environment.
        @param spread hand spread in radians; defaults to the current spread
        @param timeout blocking execution timeout, in seconds
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
        """Close the hand with a fixed spread.
        This function blocks until the hand has reached the desired
        configuration or a timeout occurs. Specifying a timeout of a finishes.
        Specifying a timeout of None blocks forever and a timeout of zero
        returns instantly. Each finger is individually collision-checked in
        simulation and will stop if it collides with the environment.
        @param spread hand spread in radians; defaults to the current spread
        @param timeout blocking execution timeout, in seconds
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

    def ResetHand(hand):
        """Reset the hand.
        This calls a low-level service to drive the fingers open with constant
        torque. Resetting the hand is the only method that is guaranteed to
        clear the fingers' breakaway state. This function blocks until the
        reset is complete.
        """
        if not hand.simulated:
            hand.controller.SendCommand('ResetHand')

    def GetState(hand):
        """Gets the current state of the hand
        """
        if hand.simulated:
            return 'done'
        else:
            # TODO: We're missing documentation here. What is the "current
            # state" of the hand? How do we interpret the return value?
            return hand.handstate_sensor.SendCommand('GetState')

    def GetStrain(hand):
        """ Gets the most recent strain sensor readings.
        @return list of strain gauge values for each finger
        """
        if not hand.simulated:
            # This is because we are overriding the force/torque sensor datatype
            sensor_data = hand.handstate_sensor.GetSensorData()
            return sensor_data.force.copy()
        else:
            return numpy.zeros(3)

    def GetBreakaway(hand):
        """Gets the most recent breakaway status of each finger.
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
        """ Gets the most recent force/torque sensor reading in the hand frame.
        Forces are specified in Newtons and torques are specified in
        Newton-meters. All readings are relative to the last time the sensor was
        tared using \ref TareForceTorqueSensor.
        @return force,torque force/torque in the hand frame
        """
        if not hand.ft_simulated:
            sensor_data = hand.ft_sensor.GetSensorData()
            return sensor_data.force, sensor_data.torque
        else:
            return numpy.zeros(3), numpy.zeros(3)

    def TareForceTorqueSensor(hand):
        """Tare the force/torque sensor.
        This is necessary before using the force/torque sensor. It is generally
        wise to tare the sensor whenever the orientation of the end-effector
        has significantly changed. This function may take several seconds to
        return as it blocks until the tare is complete.
        complete.
        """
        if not hand.ft_simulated:
            hand.ft_sensor.SendCommand('Tare')

    def _GetJointFromName(self, name):
        robot = self.manipulator.GetRobot()
        full_name = '/{:s}/{:s}'.format(self.manipulator.GetName(), name)
        return robot.GetJoint(full_name)
