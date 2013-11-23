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

import numpy, openravepy
from manipulator import Manipulator
from prpy.clone import Clone, Cloned
from .. import util
from .. import exceptions

class WAM(Manipulator):
    def __init__(self, sim, owd_namespace,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        self.controller = self.GetRobot().AttachController(name=self.GetName(),
            args='OWDController {0:s} {1:s}'.format('prpy', owd_namespace),
            dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)

        # Load the IK database.
        robot = self.GetRobot()
        if iktype is not None:
            with robot:
                self.SetActive()
                self.ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
                if not self.ikmodel.load():
                    self.ikmodel.autogenerate()

        # Enable servo motions in simulation mode.
        from prpy.simulation import ServoSimulator
        self.servo_simulator = ServoSimulator(self, rate=20, watchdog_timeout=0.1)

    def CloneBindings(self, parent):
        self.__init__(True, None)

    def SetStiffness(manipulator, stiffness):
        """
        Set the WAM's stiffness. This enables or disables gravity compensation.
        @param stiffness value between 0.0 and 1.0
        """
        if not (0 <= stiffness <= 1):
            raise Exception('Stiffness must in the range [0, 1]; got %f.' % stiffness)

        if not manipulator.simulated:
            manipulator.controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

    def Servo(manipulator, velocities):
        """
        Servo with an instantaneous vector joint velocities.
        @param joint velocities
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
        Servo's the WAM to the target taking the duration passed to it
        @param target dofs
        @param duration of the full servo
        @param timeStep
        @param collisionChecking
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

    def MoveUntilTouch(manipulator, direction, distance, max_force=5, ignore_collisions = None, **kw_args):
        """
        Execute a straight move-until-touch action. This action stops when a
        sufficient force is is felt or the manipulator moves the maximum distance.
        @param direction unit vector for the direction of motion in the world frame
        @param distance maximum distance in meters
        @param max_force maximum force in Newtons
        @param execute optionally execute the trajectory
        @param **kw_args planner parameters
        @return felt_force flag indicating whether we felt a force.
        """
        
        ignore_col_obj_oldstate = []
        if ignore_collisions is None:
            ignore_collisions = []

        for ignore_col_with in ignore_collisions:
            ignore_col_obj_oldstate.append(ignore_col_with.IsEnabled())
            ignore_col_with.Enable(False)


        with manipulator.GetRobot().GetEnv():
            manipulator.GetRobot().GetController().SimulationStep(0)

            # Compute the expected force direction in the hand frame.
            direction = numpy.array(direction)
            hand_pose = manipulator.GetEndEffectorTransform()
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            with manipulator.GetRobot():
                old_active_manipulator = manipulator.GetRobot().GetActiveManipulator()
                manipulator.SetActive()
                traj = manipulator.PlanToEndEffectorOffset(direction, distance, execute=False, **kw_args)
                traj = manipulator.GetRobot().BlendTrajectory(traj)
                traj = manipulator.GetRobot().RetimeTrajectory(traj, stop_on_ft=True, force_direction=force_direction,
                                                           force_magnitude=max_force, torque=[100,100,100])


        try:
            if not manipulator.simulated:
                manipulator.hand.TareForceTorqueSensor()
                manipulator.GetRobot().ExecuteTrajectory(traj, execute=True, retime=False, blend=False)
                for (ignore_col_with, oldstate) in zip(ignore_collisions, ignore_col_obj_oldstate):
                    ignore_col_with.Enable(oldstate)
            else:
                collided_with_obj = False
                traj_duration = traj.GetDuration()
                delta_t = 0.01

                traj_config_spec = traj.GetConfigurationSpecification()
                traj_angle_group = traj_config_spec.GetGroupFromName('joint_values')
                path_config_spec = openravepy.ConfigurationSpecification()
#                path_config_spec.AddDeltaTimeGroup()
                path_config_spec.AddGroup(traj_angle_group.name, traj_angle_group.dof, '')
#                path_config_spec.AddDerivativeGroups(1, False);
#                path_config_spec.AddDerivativeGroups(2, False);
#                path_config_spec.AddGroup('owd_blend_radius', 1, 'next')
                path_config_spec.ResetGroupOffsets()

                new_traj = openravepy.RaveCreateTrajectory(manipulator.GetRobot().GetEnv(), '')
                new_traj.Init(path_config_spec)

                for (ignore_col_with, oldstate) in zip(ignore_collisions, ignore_col_obj_oldstate):
                    ignore_col_with.Enable(oldstate)
                
                with manipulator.GetRobot():
                    manipulator.SetActive()
                    waypoint_ind = 0
                    for t in numpy.arange(0, traj_duration, delta_t):
                        traj_sample = traj.Sample(t)

                        waypoint = traj_config_spec.ExtractJointValues(traj_sample, manipulator.GetRobot(), manipulator.GetArmIndices())
                        manipulator.SetDOFValues(waypoint)
                        if manipulator.GetRobot().GetEnv().CheckCollision(manipulator.GetRobot()):
                            collided_with_obj = True
                            break
                        else:
                            #waypoint = numpy.append(waypoint,t)
                            new_traj.Insert(int(waypoint_ind), waypoint, path_config_spec)
                            waypoint_ind += 1
                    print 'execution time!!!'
                    new_traj = manipulator.GetRobot().BlendTrajectory(new_traj)
                    new_traj = manipulator.GetRobot().RetimeTrajectory(new_traj)
                manipulator.GetRobot().ExecuteTrajectory(new_traj, execute = True, retime=False, blend=False)

            return collided_with_obj
        # Trajectory is aborted by OWD because we felt a force.
        except exceptions.TrajectoryAborted:
            return True
