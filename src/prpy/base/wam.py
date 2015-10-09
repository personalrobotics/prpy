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

import logging
import numpy
import openravepy
import warnings
from manipulator import Manipulator
from prpy.clone import Clone
from .. import util
from .. import exceptions

logger = logging.getLogger('wam')

class WAM(Manipulator):
    def __init__(self, sim, owd_namespace,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        self._iktype = iktype

        if iktype is not None:
            self._SetupIK(iktype)

        # Enable servo motions in simulation mode.
        self.controller = self.GetRobot().AttachController(name=self.GetName(),
            args='OWDController {0:s} {1:s}'.format('prpy', owd_namespace),
            dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim
        )

        if sim:
            from prpy.simulation import ServoSimulator
            self.servo_simulator = ServoSimulator(
                self, rate=20, watchdog_timeout=0.1)

    def CloneBindings(self, parent):
        Manipulator.CloneBindings(self, parent)

        self.simulated = True
        self.controller = None
        self._iktype = parent._iktype

        if self._iktype is not None:
            self._SetupIK(self._iktype)

    def _SetupIK(self, iktype):
        from openravepy.databases.inversekinematics import InverseKinematicsModel

        robot = self.GetRobot()
        self.ikmodel = InverseKinematicsModel(robot=robot, manip=self,
                                              iktype=iktype)
        if not self.ikmodel.load():
            self.ikmodel.generate(iktype=iktype, precision=4,
                                  freeindices=[ self.GetIndices()[2] ])
            self.ikmodel.save()

    def SetStiffness(manipulator, stiffness):
        """Set the WAM's stiffness.
        Stiffness 0 is gravity compensation and stiffness 1 is position
        control. Values between 0 and 1 are experimental.
        @param stiffness value between 0.0 and 1.0
        """
        if not (0 <= stiffness <= 1):
            raise Exception('Stiffness must in the range [0, 1]; got %f.' % stiffness)

        if not manipulator.simulated:
            manipulator.controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

    def SetTrajectoryExecutionOptions(self, traj, stop_on_stall=False,
            stop_on_ft=False, force_magnitude=None, force_direction=None,
            torque=None):
        """Set OWD's trajectory execution options on trajectory.
        @param stop_on_stall aborts the trajectory if the arm stalls
        @param stop_on_ft aborts the trajectory if the force/torque
                          sensor reports a force or torque that exceeds
                          threshold specified by the force_magnitude,
                          force_direction, and torque options
        @param force_magnitude force threshold value, in Newtons
        @param force_direction unit vector in the force/torque coordinate
                               frame, which is int he same orientation as the
                               hand frame.
        @param torque vector of the three torques
        """
        util.SetTrajectoryTags(traj, {
            'owd_options': {
                'stop_on_stall': bool(stop_on_stall),
                'stop_on_ft': bool(stop_on_ft),
                'force_magnitude': float(force_magnitude),
                'force_direction': list(force_direction),
                'torque': list(torque),
        }}, append=True)

    def Servo(manipulator, velocities):
        """Servo with a vector of instantaneous joint velocities.
        @param velocities joint velocities, in radians per second
        """
        num_dof = len(manipulator.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. '
                             'Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if not manipulator.simulated:
            manipulator.controller.SendCommand('Servo ' + ' '.join([ str(qdot) for qdot in velocities ]))
        else:
            manipulator.controller.Reset(0)
            manipulator.servo_simulator.SetVelocity(velocities)

    def ServoTo(manipulator, target, duration, timeStep=0.05, collisionChecking=True):
        """Servo the arm towards a target configuration over some duration.
        Servos the arm towards a target configuration with a constant joint
        velocity. This function uses the \ref Servo command to control the arm
        and must be called repeatidly until the arm reaches the goal. If \tt
        collisionChecking is enabled, then the servo will terminate and return
        False if a collision is detected with the simulation environment.
        @param target desired configuration
        @param duration duration in seconds
        @param timestep period of the control loop, in seconds
        @param collisionchecking check collisions in the simulation environment
        @return whether the servo was successful
        """
        steps = int(math.ceil(duration/timeStep))
        original_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
        velocity = numpy.array(target-manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices()))
        velocities = v/steps#[v/steps for v in velocity]
        inCollision = False 
        if collisionChecking:
            inCollision = manipulator.CollisionCheck(target)

        if not inCollision:
            for i in range(1,steps):
                import time
                manipulator.Servo(velocities)
                time.sleep(timeStep)
            manipulator.Servo([0] * len(manipulator.GetArmIndices()))
            new_dofs = manipulator.GetRobot().GetDOFValues(manipulator.GetArmIndices())
            return True
        else:
            return False

    def GetVelocityLimits(self, openrave=None, owd=None):
        """Get the OpenRAVE and OWD joint velocity limits.
        This function checks both the OpenRAVE and OWD joint velocity limits.
        If they do not match, a warning is printed and the minimum value is
        returned.
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        @return list of velocity limits, in radians per second
        """
        if openrave is not None or owd is not None:
            warnings.warn(
                'The "openrave" and "owd" flags are deprecated in'
                ' GetVelocityLimits and will be removed in a future version.',
                DeprecationWarning)

        return Manipulator.GetVelocityLimits(self)
        
    def SetVelocityLimits(self, velocity_limits, min_accel_time,
                          openrave=True, owd=True):
        """Change the OpenRAVE and OWD joint velocity limits.
        Joint velocities that exceed these limits will trigger a velocity fault.
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
        """Gets the status of the current (or previous) trajectory executed by OWD.
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
        """Clears the current trajectory execution status.
        This resets the output of \ref GetTrajectoryStatus.
        """
        if not manipulator.simulated:
            manipulator.controller.SendCommand('ClearStatus')

    def MoveUntilTouch(manipulator, direction, distance, max_distance=None,
                       max_force=5.0, max_torque=None, ignore_collisions=None,
                       velocity_limit_scale=0.25, **kw_args):
        """Execute a straight move-until-touch action.
        This action stops when a sufficient force is is felt or the manipulator
        moves the maximum distance. The motion is considered successful if the
        end-effector moves at least distance. In simulation, a move-until-touch
        action proceeds until the end-effector collids with the environment.

        @param direction unit vector for the direction of motion in the world frame
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param max_force maximum force in Newtons
        @param max_torque maximum torque in Newton-Meters
        @param ignore_collisions collisions with these objects are ignored when planning the path, e.g. the object you think you will touch
        @param velocity_limit_scale A multiplier to use to scale velocity limits when executing MoveUntilTouch ( < 1 in most cases).           
        @param **kw_args planner parameters
        @return felt_force flag indicating whether we felt a force.
        """
        from contextlib import nested
        from openravepy import CollisionReport, KinBody, Robot, RaveCreateTrajectory
        from ..planning.exceptions import CollisionPlanningError

        delta_t = 0.01

        robot = manipulator.GetRobot()
        env = robot.GetEnv()
        dof_indices = manipulator.GetArmIndices()

        direction = numpy.array(direction, dtype='float')

        # Default argument values.
        if max_distance is None:
            max_distance = 1.
            warnings.warn(
                'MoveUntilTouch now requires the "max_distance" argument.'
                ' This will be an error in the future.',
                DeprecationWarning)

        if max_torque is None:
            max_torque = numpy.array([100.0, 100.0, 100.0])

        if ignore_collisions is None:
            ignore_collisions = []

        with env:
            # Compute the expected force direction in the hand frame.
            hand_pose = manipulator.GetEndEffectorTransform()
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            # Disable the KinBodies listed in ignore_collisions. We backup the
            # "enabled" state of all KinBodies so we can restore them later.
            body_savers = [
                body.CreateKinBodyStateSaver() for body in ignore_collisions]
            robot_saver = robot.CreateRobotStateSaver(
                  Robot.SaveParameters.ActiveDOF
                | Robot.SaveParameters.ActiveManipulator
                | Robot.SaveParameters.LinkTransformation)

            with robot_saver, nested(*body_savers) as f:
                manipulator.SetActive()
                robot_cspec = robot.GetActiveConfigurationSpecification()

                for body in ignore_collisions:
                    body.Enable(False)

                path = robot.PlanToEndEffectorOffset(direction=direction,
                    distance=distance, max_distance=max_distance, **kw_args)

        # Execute on the real robot by tagging the trajectory with options that
        # tell the controller to stop on force/torque input.
        if not manipulator.simulated:
            manipulator.SetTrajectoryExecutionOptions(path, stop_on_ft=True,
                force_direction=force_direction, force_magnitude=max_force,
                torque=max_torque)

            manipulator.hand.TareForceTorqueSensor()

            try:
                with robot.CreateRobotStateSaver(Robot.SaveParameters.JointMaxVelocityAndAcceleration):
                    manipulator.SetVelocityLimits(velocity_limit_scale*vl, 0.5)
                    robot.ExecutePath(path)
                return False
            except exceptions.TrajectoryAborted as e:
                logger.warn('MoveUntilTouch aborted: %s', str(e))
                return True
        # Forward-simulate the motion until it hits an object.
        else:
            traj = robot.PostProcessPath(path)
            is_collision = False

            traj_cspec = traj.GetConfigurationSpecification()
            new_traj = RaveCreateTrajectory(env, '')
            new_traj.Init(traj_cspec)

            robot_saver = robot.CreateRobotStateSaver(
                Robot.SaveParameters.LinkTransformation)
            
            with env, robot_saver:
                for t in numpy.arange(0, traj.GetDuration(), delta_t):
                    waypoint = traj.Sample(t)

                    dof_values = robot_cspec.ExtractJointValues(
                        waypoint, robot, dof_indices, 0)
                    manipulator.SetDOFValues(dof_values)

                    # Terminate if we detect collision with the environment.
                    report = CollisionReport()
                    if env.CheckCollision(robot, report=report):
                        logger.info('Terminated from collision: %s',
                            str(CollisionPlanningError.FromReport(report)))
                        is_collision = True
                        break
                    elif robot.CheckSelfCollision(report=report):
                        logger.info('Terminated from self-collision: %s',
                            str(CollisionPlanningError.FromReport(report)))
                        is_collision = True
                        break

                    # Build the output trajectory that stops in contact.
                    if new_traj.GetNumWaypoints() == 0:
                        traj_cspec.InsertDeltaTime(waypoint, 0.)
                    else:
                        traj_cspec.InsertDeltaTime(waypoint, delta_t)
                    
                    new_traj.Insert(new_traj.GetNumWaypoints(), waypoint)

            if new_traj.GetNumWaypoints() > 0:
                robot.ExecuteTrajectory(new_traj)

            return is_collision
