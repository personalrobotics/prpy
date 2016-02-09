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

import numpy
import openravepy
from .. import util
from ..exceptions import PrPyException
from endeffector import EndEffector

class MicoHand(EndEffector):
    def __init__(self, sim, manipulator):
        EndEffector.__init__(self, manipulator)

        robot = manipulator.GetRobot()
        env = robot.GetEnv()

        self.simulated = sim

        with env:
            accel_limits = robot.GetDOFAccelerationLimits()
            accel_limits[self.GetIndices()] = 1.
            robot.SetDOFAccelerationLimits(accel_limits)

        if sim:
            robot = manipulator.GetRobot()
            self.controller = robot.AttachController(
                self.GetName(), '', self.GetIndices(), 0, True)

    def CloneBindings(self, parent):
        super(MicoHand, self).CloneBindings(parent)

        self.simulated = True

    def MoveHand(self, f1, f2, timeout=None):
        """
        Change the hand preshape. This function blocks until trajectory
        execution finishes. This can be changed by changing the timeout
        parameter to a maximum number of seconds. Pass zero to return
        instantantly.

        @param f1 finger 1 angle
        @param f2 finger 2 angle
        @param timeout blocking execution timeout
        """

        from openravepy import PlannerStatus

        robot = self.GetParent()

        with robot.GetEnv():
            sp = openravepy.Robot.SaveParameters
            with robot.CreateRobotStateSaver(sp.ActiveDOF):
                robot.SetActiveDOFs(self.GetIndices())
                cspec = robot.GetActiveConfigurationSpecification('linear')
                current_preshape = robot.GetActiveDOFValues()

            # Default any None's to the current DOF values.
            desired_preshape = current_preshape.copy()
            if f1 is not None: desired_preshape[0] = f1
            if f2 is not None: desired_preshape[1] = f2

        # Create a two waypoint trajectory to the target configuration.
        traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
        traj.Init(cspec)
        traj.Insert(0, current_preshape)
        traj.Insert(1, desired_preshape)

        # Time the trajectory so we can execute it.
        result = openravepy.planningutils.RetimeTrajectory(
            traj, False, 1., 1., 'LinearTrajectoryRetimer')

        if result not in [ PlannerStatus.HasSolution,
                           PlannerStatus.InterruptedWithSolution ]:
            raise PrPyException('Failed timing finger trajectory.')

        # Execute the trajectory.
        return robot.ExecuteTrajectory(traj)
       
    def OpenHand(self, value=0., timeout=None):
        """
        Open the hand.
        @param timeout blocking execution timeout
        """
        if self.simulated:
            robot = self.manipulator.GetRobot()
            p = openravepy.KinBody.SaveParameters

            with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
                self.manipulator.SetActive()
                robot.task_manipulation.ReleaseFingers()

            if timeout:
                robot.WaitForController(timeout)

            return None
        else:
            return self.MoveHand(f1=value, f2=value, timeout=timeout)

    def CloseHand(self, value=0.8, timeout=None):
        """ Close the hand.
        @param timeout blocking execution timeout
        """
        return self.MoveHand(f1=value, f2=value, timeout=timeout)

    def CloseHandTight(self, value=1.2, timeout=None):
        """ Close the hand tightly.
        @param timeout blocking execution timeout
        """
        return self.CloseHand(value=value, timeout=timeout)
