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
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

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
        self.env = openravepy.Environment()

    def __str__(self):
        return 'mk'

    def GetStraightVelocity(self, manip, velocity, initial_hand_pose, nullspace_fn, step_size):
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
        error_ori = min(choices_ori, key=lambda q: numpy.linalg.norm(q))

        # Jacobian pseudo-inverse.
        jacobian_spatial = manip.CalculateJacobian()
        jacobian_angular = manip.CalculateRotationJacobian()
        jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))
        jacobian_pinv = numpy.linalg.pinv(jacobian)

        # Null-space projector
        nullspace_projector = numpy.eye(jacobian.shape[1]) - numpy.dot(jacobian_pinv, jacobian)
        nullspace_goal = nullspace_fn(robot)
        pose_error = numpy.hstack((error_pos, error_ori))
        return numpy.dot(jacobian_pinv, pose_error) + numpy.dot(nullspace_projector, nullspace_goal)

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, max_distance=None,
                                nullspace=JointLimitAvoidance, timelimit=2.5, step_size=0.001,
                                position_tolerance=0.01, angular_tolerance=0.15, **kw_args):
        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')
        elif max_distance is None or max_distance < distance:
            raise ValueError('Max distance is less than minimum distance.')
        elif step_size <= 0:
            raise ValueError('Step size must be positive.')
        elif position_tolerance < 0:
            raise ValueError('Position tolerance must be non-negative.')
        elif angular_tolerance < 0:
            raise ValueError('Angular tolerance must be non-negative.')

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
            try:
                while current_distance < max_distance:
                    # Check for a timeout.
                    current_time = time.time()
                    if timelimit is not None and current_time - start_time > timelimit:
                        raise PlanningError('Reached time limit.')

                    # Compute joint velocities using the Jacobian pseudoinverse.
                    q_dot = self.GetStraightVelocity(manip, direction, initial_pose, nullspace, step_size)
                    q += q_dot
                    robot.SetDOFValues(q, active_dof_indices)
                    traj.Insert(traj.GetNumWaypoints(), q)

                    # Check for collisions.
                    if self.env.CheckCollision(robot):
                        raise PlanningError('Encountered collision.')
                    elif robot.CheckSelfCollision():
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
                    if numpy.linalg.norm(offset_angle) > angular_tolerance:
                        raise PlanningError('Deviated from orientation constraint.')

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

        return traj

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        raise UnsupportedPlanningError('PlanToConfiguration not implemented for MK planner')

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, psample=0.1, **kw_args):
        raise UnsupportedPlanningError('PlanToEndEffectorPose not implemented for MK planner')

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        raise UnsupportedPlanningError('PlanToTSR not implemented for MK planner')
