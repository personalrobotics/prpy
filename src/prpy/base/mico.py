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

import numpy, openravepy
from manipulator import Manipulator
from prpy.clone import Clone, Cloned
from .. import util
from .. import exceptions

class Mico(Manipulator):
    def __init__(self, sim, controller_namespace,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        # TODO: learn how to attach an ideal controller controller
        #self.controller = self.GetRobot().AttachController(name=self.GetName(),
            #args='OWDController {0:s} {1:s}'.format('prpy', owd_namespace),
            #dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)
        self.controller = self.GetRobot().AttachController(name=self.GetName(),
            args='MicoController {0:s} {1:s}'.format('prpy', controller_namespace),
            dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)
            
	#self.hand_controller = self.GetRobot().AttachController(name=self.GetRobot().GetName(),
            #args='MicoHandController {0:s} {1:s}'.format('prpy', '/mico_hand_controller'),
            #dof_indices=self.GetGripperIndices(), affine_dofs=0, simulated=sim)

        # Load the IK database.
        robot = self.GetRobot()
        
        '''
        if iktype is not None:
            with robot:
                self.SetActive()
                self.ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
                if not self.ikmodel.load():
                    self.ikmodel.autogenerate()
	'''
        # Enable servo motions in simulation mode.
        from prpy.simulation import ServoSimulator
        self.servo_simulator = ServoSimulator(self, rate=20, watchdog_timeout=0.1)

    def CloneBindings(self, parent):
        self.__init__(True, None)

    def PlanToNamedConfiguration(self, name, execute=True, **kw_args):
        """
        Plan this arm to saved configuration stored in robot.configurations by
        ignoring any other DOFs specified in the named configuration.
        @param name name of a saved configuration
        @param **kw_args optional arguments passed to PlanToConfiguration
        @returns traj trajectory
        """
        robot = self.GetRobot()
        saved_dof_indices, saved_dof_values = robot.configurations.get_configuration(name)

        with Clone(robot.GetEnv()):
            Cloned(self).SetActive()
            arm_dof_indices = Cloned(robot).GetActiveDOFIndices()
            arm_dof_values = Cloned(robot).GetActiveDOFValues()

            for arm_dof_index, arm_dof_value in zip(saved_dof_indices, saved_dof_values):
                if arm_dof_index in arm_dof_indices:
                    i = list(arm_dof_indices).index(arm_dof_index)
                    arm_dof_values[i] = arm_dof_value

            traj = Cloned(robot).PlanToConfiguration(arm_dof_values, execute=False, **kw_args)

            # Clone the trajectory back into the live environment
            live_traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), traj.GetXMLId())
            live_traj.Clone(traj, 0)

        if execute:
            return robot.ExecuteTrajectory(live_traj, **kw_args)
        else:
            return live_traj

    def SetStiffness(manipulator, stiffness):
        """
        Set the Mico's stiffness. This enables or disables gravity compensation.
        Values between 0 and 1 are experimental.
        @param stiffness value between 0.0 and 1.0
        """
        if not (0 <= stiffness <= 1):
            raise Exception('Stiffness must in the range [0, 1]; got %f.' % stiffness)

        if not manipulator.simulated:
            manipulator.controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

    def Servo(manipulator, velocities):
        """
        Servo with an instantaneous vector of joint velocities.
        @param velocities instantaneous joint velocities in radians per second
        """
        num_dof = len(manipulator.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if not manipulator.simulated:
            manipulator.controller.SendCommand('Servo ' + ' '.join([ str(qdot) for qdot in velocities ]))
        else:
            manipulator.controller.Reset(0)
            manipulator.servo_simulator.SetVelocity(velocities)

    def ServoTo(manipulator, target, duration, timeStep = 0.05, collisionChecking= True):
        """
        Servo's the Mico to the target taking the duration passed to it
        @param target desired joint angles
        @param duration duration in seconds
        @param timestep period of the control loop
        @param collisionchecking check collisions in the simulation environment
        """
        steps = int(math.ceil(duration/timeStep))
        original_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
        velocity = numpy.array(target-manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices()))
        velocities = v/steps#[v/steps for v in velocity]
        inCollision = False 
        if collisionChecking==True:
            inCollision = manipulator.CollisionCheck(target)
        if inCollision == False:       
            for i in range(1,steps):
                manipulator.Servo(velocities)
                time.sleep(timeStep)
            manipulator.Servo([0] * len(manipulator.GetArmIndices()))
            new_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
            return True
        return False

    def SetVelocityLimits(self, velocity_limits, min_accel_time,
                          openrave=True, owd=True):
        """
        Change the OpenRAVE and OWD joint velocity limits. Joint velocities
        that exceed these limits will trigger a velocity fault.
        @param velocity_limits vector of joint velocity limits in radians per second
        @param min_accel_time minimum acceleration time used to compute acceleration limits
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        """
        # Update the OpenRAVE limits.
        if openrave:
            Manipulator.SetVelocityLimits(self, velocity_limits, min_accel_time)

        # Update the OWD limits.
        if owd and not self.simulated:
            args  = [ 'SetSpeed' ]
            args += [ str(min_accel_time) ]
            args += [ str(velocity) for velocity in velocity_limits ]
            args_str = ' '.join(args)
            self.controller.SendCommand(args_str)

    def GetTrajectoryStatus(manipulator):
        """
        Gets the status of the current (or previous) trajectory executed by the
        controller.
        @return status of the current (or previous) trajectory executed
        """
        if not manipulator.simulated:
            return manipulator.controller.SendCommand('GetStatus')
        else:
            if manipulator.controller.IsDone():
                return 'done'
            else:
                return 'active'

    def ClearTrajectoryStatus(manipulator):
        """
        Clears the current trajectory execution status.
        """
        if not manipulator.simulated:
            manipulator.controller.SendCommand('ClearStatus')

    