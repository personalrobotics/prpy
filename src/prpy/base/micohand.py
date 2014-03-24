#!/usr/bin/env python

# Copyright (c) 2014, Carnegie Mellon University
# All rights reserved.
# Authors: Tekin Mericli <tekin@cmu.edu>
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

class MicoHand(EndEffector):
    def __init__(self, sim, manipulator, controller_namespace, hand_namespace):
        EndEffector.__init__(self, manipulator)

        self.simulated = sim

        # Hand controller
        robot = self.manipulator.GetRobot()
        env = robot.GetEnv()
        self.controller = robot.AttachController(name=self.GetName(),
            args='MicoHandController {0:s} {1:s}'.format('prpy', hand_namespace),
            dof_indices=self.GetIndices(), affine_dofs=0, simulated=sim)

    def MoveHand(hand, f1=None, f2=None, spread=None, timeout=None):
        """
        Change the hand preshape. This function blocks until trajectory execution
        finishes. This can be changed by changing the timeout parameter to a
        maximum number of seconds. Pass zero to return instantantly.
        @param f1 finger 1 angle
        @param f2 finger 2 angle
        @param spread spread angle
        @param timeout blocking execution timeout
        """
        # Default any None's to the current DOF values.
        preshape = hand.GetDOFValues()
        if f1     is not None: preshape[0] = f1
        if f2     is not None: preshape[1] = f2
        if spread is not None: preshape[2] = spread

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
            hand.MoveHand(f1=0.0, f2=0.0, spread=spread, timeout=timeout)

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
            hand.MoveHand(f1=3.2, f2=3.2, spread=spread, timeout=timeout)

    def ResetHand(hand):
        """
        Reset the hand
        """
        if not hand.simulated:
            hand.controller.SendCommand('ResetHand')

    def GetState(hand):
        """
        Gets the current state of the hand
        """
        if hand.simulated:
            return 'done'
        else:
            return hand.handstate_sensor.SendCommand('GetState')
            