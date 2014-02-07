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

def create_affine_trajectory(robot, poses):
    doft = openravepy.DOFAffine.Transform
    cspec = openravepy.RaveGetAffineConfigurationSpecification(doft, robot)
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
    traj.Init(cspec)

    for iwaypoint, pose in enumerate(poses):
        waypoint = openravepy.RaveGetAffineDOFValuesFromTransform(pose, doft)
        traj.Insert(iwaypoint, waypoint)

    return traj


class MobileBase(object):
    def __init__(self, sim, robot):
        self.simulated = sim
        self.robot = robot

    def CloneBindings(self, parent):
        pass

    def Forward(self, meters, execute=True, **kw_args):
        """
        Drives the robot forward the desired distance
        Note: Only implemented in simulation. Derived robots should implement this method.
        @param meters the distance to drive the robot
        @param timout duration to wait for execution
        """
        if self.simulated:
            with self.robot.GetEnv():
                start_pose = self.robot.GetTransform()
                offset_pose = numpy.eye(4)
                offset_pose[0, 3] = meters
                goal_pose = numpy.dot(start_pose, offset_pose)

            traj = create_affine_trajectory(self.robot, [ start_pose, goal_pose ])
            if execute:
                return self.robot.ExecuteTrajectory(traj, **kw_args)
            else:
                return traj
        else:
            raise NotImplementedError('DriveForward is not implemented')

    def Rotate(self, angle_rad, execute=True, **kw_args):
        """
        Rotates the robot the desired distance
        @param angle_rad the number of radians to rotate
        @param timeout duration to wait for execution
        """
        if self.simulated:
            with self.robot.GetEnv():
                start_pose = self.robot.GetTransform()

            relative_pose = openravepy.matrixFromAxisAngle([ 0., 0., angle_rad ])
            goal_pose = numpy.dot(start_pose, relative_pose)

            traj = create_affine_trajectory(self.robot, [ start_pose, goal_pose ])
            if execute:
                return self.robot.ExecuteTrajectory(traj, **kw_args)
            else:
                return traj
        else:
            raise NotImplementedError('Rotate is not implemented')

    def DriveStraightUntilForce(self, direction, velocity=0.1, force_threshold=3.0,
                                max_distance=None, timeout=None, left_arm=True, right_arm=True):
        """
        Drive the base in a direction until a force/torque sensor feels a force. The
        base first turns to face the desired direction, then drives forward at the
        specified velocity. The action terminates when max_distance is reached, the
        timeout is exceeded, or if a force is felt. The maximum distance and timeout
        can be disabled by setting the corresponding parameters to None.
        @param direction forward direction of motion in the world frame
        @param velocity desired forward velocity
        @param force_threshold threshold force in Newtons
        @param max_distance maximum distance in meters
        @param timeout maximum duration in seconds
        @param left_arm flag to use the left force/torque sensor
        @param right_arm flag to use the right force/torque sensor
        @return felt_force flag indicating whether the action felt a force
        """
        if self.simulated:
            raise NotImplementedError('DriveStraightUntilForce does not work in simulation')
        else:
            raise NotImplementedError('DriveStraightUntilForce is not implemented')
