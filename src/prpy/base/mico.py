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

import numpy, openravepy, rospy
from manipulator import Manipulator
from prpy.clone import Clone
from .. import util
from .. import exceptions
from IPython import embed

class Mico(Manipulator):
    def _load_controllers(self, controllers, timeout=10):
        from controller_manager_msgs.srv import SwitchController, ListControllers
        """Load a list of ros_control controllers by name."""

        rospy.wait_for_service('controller_manager/switch_controller', timeout=10)
        rospy.wait_for_service('controller_manager/list_controllers', timeout=10)

        switch_controllers = rospy.ServiceProxy(
                'controller_manager/switch_controller', SwitchController)
        list_controllers = rospy.ServiceProxy(
                'controller_manager/list_controllers', ListControllers)

        running = set([c.name for c in list_controllers().controller  if c.state == "running"])
        controllers = set(controllers)
        switch_controllers(list(controllers - running), list(running - controllers), 2)
    
    def _unload_controllers(self, controllers):
        from controller_manager_msgs.srv import SwitchController, ListControllers
        """Unload a list of ros_control controllers by name"""

        rospy.wait_for_service('controller_manager/switch_controller')
        rospy.wait_for_service('controller_manager/list_controllers')

        switch_controllers = rospy.ServiceProxy(
                'controller_manager/switch_controller', SwitchController)
        list_controllers = rospy.ServiceProxy(
                'controller_manager/list_controllers', ListControllers)

        running = set([c.name for c in list_controllers().controller
            if c.state == "running"])
        controllers = set(controllers)
        switch_controllers([], list(controllers & running), 2)


    def __init__(self, sim, controller_namespace,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        robot = self.GetRobot()

        if sim:
            self.controller = self.GetRobot().AttachController(name=self.GetName(),
            args='idealcontroller'.format('prpy', controller_namespace),
            dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)        
        else:
            self.or_physical_controller = ('roscontroller openrave {0} 1'.format(rospy.get_namespace()))
            self.ros_controllers = [
                "traj_controller",
                "joint_state_controller",
            ]
            or_controller_string = self.or_physical_controller
            self._load_controllers(self.ros_controllers)
            env = robot.GetEnv()
            with env:
                self.controller = openravepy.RaveCreateController(env,
                    or_controller_string)
            m = robot.GetActiveDOFIndices()
            c = self.controller
            robot.SetController(c, m, 0)

        # Enable servo motions in simulation mode.
        if sim:
            from prpy.simulation import ServoSimulator

            self.servo_simulator = ServoSimulator(self, rate=20, watchdog_timeout=0.1)

    def CloneBindings(self, parent):
        self.__init__(True, None)

    def Servo(self, velocities):
        """
        Servo with an instantaneous vector of joint velocities.
        @param velocities instantaneous joint velocities in radians per second
        """
        num_dof = len(self.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if self.simulated:
            self.controller.Reset(0)
            self.servo_simulator.SetVelocity(velocities)
        else:
            raise NotImplementedError('Servo is not implemented on the real robot.')
