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

import logging, numpy, openravepy, time
from ..util import SetTrajectoryTags
from base import (BasePlanner, PlanningError, UnsupportedPlanningError,
                  PlanningMethod, Tags)

logger = logging.getLogger('planning')

def DoNothing(robot):
    return numpy.zeros(robot.GetActiveDOF())

# Based on Moslem Kazemi's code from ARM-S.
def JointLimitAvoidance(robot, limit_tolerance=0.2, gain=100):
    q = robot.GetActiveDOFValues()
    q_min, q_max = robot.GetDOFLimits(robot.GetActiveDOFIndices())

    num_dofs = robot.GetActiveDOF()
    q_dot = numpy.zeros(num_dofs)
    for i in xrange(num_dofs):
        max_limit_dist = q_max[i] - q[i]
        min_limit_dist = q_min[i] - q[i]

        if max_limit_dist < limit_tolerance:
            q_dot[i] = -gain * (max_limit_dist - limit_tolerance) ** 2
        elif min_limit_dist > -limit_tolerance:
            q_dot[i] =  gain * (min_limit_dist + limit_tolerance) ** 2
        else:
            q_dot[i] = 0

    return q_dot

class MKPlanner(BasePlanner):
    def __init__(self):
        super(MKPlanner, self).__init__()

    def __str__(self):
        return 'MKPlanner'

    #Calculates a change of joint angles to maintain the same orientation, and move the position forward slightly
        ### NOTE: The sign_flipper is a hack
        ### Sometimes, it seems changing the direction of the error term caused it to succeed
        ### sign_flipper is monitered by the planner. If the orientation error starts increasing, it flips the sign of the error term
    def GetStraightVelocity(self, manip, velocity, initial_hand_pose, nullspace_fn, step_size, sign_flipper = 1):
        robot = manip.GetRobot()
        current_hand_pose = manip.GetEndEffectorTransform()
        initial_position = initial_hand_pose[0:3, 3]
        current_position = current_hand_pose[0:3, 3]

        # Simulate a position goal step_size distance further along the velocity vector.
        moved_already = velocity * numpy.dot(current_position - initial_position, velocity)
        desired_position = initial_position + moved_already + velocity * step_size
        error_pos = desired_position - current_position

        # Append the desired quaternion to create the error vector. There is a
        # sign ambiguity on quaternions, so we'll always choose the shortest path.
        initial_ori = openravepy.quatFromRotationMatrix(initial_hand_pose)
        current_ori = openravepy.quatFromRotationMatrix(current_hand_pose)
        choices_ori = [ current_ori - initial_ori, current_ori + initial_ori ]
        error_ori = sign_flipper*min(choices_ori, key=lambda q: numpy.linalg.norm(q))

        # Jacobian pseudo-inverse.
        jacobian_spatial = manip.CalculateJacobian()
        jacobian_angular = manip.CalculateRotationJacobian() #this function seems very buggy/wrong
        jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))
        jacobian_pinv = numpy.linalg.pinv(jacobian)

        # Null-space projector
        nullspace_projector = numpy.eye(jacobian.shape[1]) - numpy.dot(jacobian_pinv, jacobian)
        nullspace_goal = nullspace_fn(robot)
        pose_error = numpy.hstack((error_pos, error_ori))

        return numpy.dot(jacobian_pinv, pose_error) + numpy.dot(nullspace_projector, nullspace_goal)

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, max_distance=None,
                                nullspace=JointLimitAvoidance, timelimit=5.0, step_size=0.001,
                                position_tolerance=0.01, angular_tolerance=0.15, **kw_args):
        """
        Plan to a desired end-effector offset with move-hand-straight
        constraint. movement less than distance will return failure. The motion
        will not move further than max_distance.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param timelimit timeout in seconds
        @param stepsize step size in meters for the Jacobian pseudoinverse controller
        @param position_tolerance constraint tolerance in meters
        @param angular_tolerance constraint tolerance in radians
        @return traj
        """
        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')
        elif max_distance is not None and max_distance < distance:
            raise ValueError('Max distance is less than minimum distance.')
        elif step_size <= 0:
            raise ValueError('Step size must be positive.')
        elif position_tolerance < 0:
            raise ValueError('Position tolerance must be non-negative.')
        elif angular_tolerance < 0:
            raise ValueError('Angular tolerance must be non-negative.')

        # save all active bodies so we only check collision with those
        active_bodies = []
        for body in self.env.GetBodies():
            if body.IsEnabled():
                active_bodies.append(body)
        

        # Normalize the direction vector.
        direction  = numpy.array(direction, dtype='float')
        direction /= numpy.linalg.norm(direction)

        # Default to moving an exact distance.
        if max_distance is None:
            max_distance = distance

        with robot:
            manip = robot.GetActiveManipulator()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            traj.Init(manip.GetArmConfigurationSpecification())

            active_dof_indices = manip.GetArmIndices()
            limits_lower, limits_upper = robot.GetDOFLimits(active_dof_indices)
            initial_pose = manip.GetEndEffectorTransform()
            q = robot.GetDOFValues(active_dof_indices)
            traj.Insert(0, q)

            start_time = time.time()
            current_distance = 0.0
            sign_flipper = 1
            last_rot_error = 9999999999.0
            try:
                while current_distance < max_distance:
                    # Check for a timeout.
                    current_time = time.time()
                    if timelimit is not None and current_time - start_time > timelimit:
                        raise PlanningError('Reached time limit.')

                    # Compute joint velocities using the Jacobian pseudoinverse.
                    q_dot = self.GetStraightVelocity(manip, direction, initial_pose, nullspace, step_size, sign_flipper=sign_flipper)
                    q += q_dot
                    robot.SetDOFValues(q, active_dof_indices)

                    # Check for collisions.
                    #if self.env.CheckCollision(robot):
                    for body in active_bodies:
                        if self.env.CheckCollision(robot, body):
                            raise PlanningError('Encountered collision.')
                    if robot.CheckSelfCollision():
                        raise PlanningError('Encountered self-collision.')
                    # Check for joint limits.
                    elif not (limits_lower < q).all() or not (q < limits_upper).all():
                        raise PlanningError('Encountered joint limit during Jacobian move.')

                    # Check our distance from the constraint.
                    current_pose = manip.GetEndEffectorTransform()
                    a = initial_pose[0:3, 3]
                    p = current_pose[0:3, 3]
                    orthogonal_proj = (a - p) - numpy.dot(a - p, direction) * direction
                    if numpy.linalg.norm(orthogonal_proj) > position_tolerance:
                        raise PlanningError('Deviated from a straight line constraint.')

                    # Check our orientation against the constraint.
                    offset_pose = numpy.dot(numpy.linalg.inv(current_pose), initial_pose)
                    offset_angle = openravepy.axisAngleFromRotationMatrix(offset_pose)
                    offset_angle_norm = numpy.linalg.norm(offset_angle)
                    if offset_angle_norm > last_rot_error + 0.0005:
                        sign_flipper *= -1
                    last_rot_error = offset_angle_norm
                    if offset_angle_norm > angular_tolerance:
                        raise PlanningError('Deviated from orientation constraint.')

                    traj.Insert(traj.GetNumWaypoints(), q)

                    # Check if we've exceeded the maximum distance by projecting our
                    # displacement along the direction.
                    hand_pose = manip.GetEndEffectorTransform()
                    displacement = hand_pose[0:3, 3] - initial_pose[0:3, 3]
                    current_distance = numpy.dot(displacement, direction)
            except PlanningError as e:
                # Throw an error if we haven't reached the minimum distance.
                if current_distance < distance:
                    raise
                # Otherwise we'll gracefully terminate.
                else:
                    logger.warning('Terminated early at distance %f < %f: %s',
                                   current_distance, max_distance, e.message)

        SetTrajectoryTags(output_traj, {Tags.CONSTRAINED: True}, append=True)
        return traj

