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

import openravepy
from manipulator import Manipulator

class Mico(Manipulator):
    def __init__(self, sim,
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        self.iktype = iktype

        robot = self.GetRobot()
        env = robot.GetEnv()

        with env:
            dof_indices = self.GetIndices()
            accel_limits = robot.GetDOFAccelerationLimits()
            accel_limits[dof_indices[0]] = 1.45
            accel_limits[dof_indices[1]] = 1.56
            accel_limits[dof_indices[2]] = 1.56
            accel_limits[dof_indices[3]] = 1.5
            accel_limits[dof_indices[4]] = 1.48
            accel_limits[dof_indices[5]] = 1.49
            robot.SetDOFAccelerationLimits(accel_limits)

        # Load or_nlopt_ik as the IK solver. Unfortunately, IKFast doesn't work
        # on the Mico.
        if iktype is not None:
            self.iksolver = openravepy.RaveCreateIkSolver(env, 'NloptIK')
            self.SetIKSolver(self.iksolver)

        # Load simulation controllers.
        if sim:
            from prpy.simulation import ServoSimulator

            self.controller = robot.AttachController(
                self.GetName(), '', self.GetArmIndices(), 0, True)
            self.servo_simulator = ServoSimulator(self, rate=20,
                                                  watchdog_timeout=0.1)

    def CloneBindings(self, parent):
        super(Mico, self).CloneBindings(parent)

        self.simulated = True
        self.iktype = parent.iktype

        self.servo_simulator = None

        # TODO: This is broken on nlopt_ik
        """
        if parent.iktype is not None:
            self.iksolver = openravepy.RaveCreateIkSolver(env, 'NloptIK')
            self.SetIKSolver(self.iksolver)
        """

    def Servo(self, velocities):
        """
        Servo with an instantaneous vector of joint velocities.
        @param velocities instantaneous joint velocities in radians per second
        """
        num_dof = len(self.GetArmIndices())

        if len(velocities) != num_dof:
            raise ValueError(
                'Incorrect number of joint velocities.'
                ' Expected {:d}; got {:d}.'.format(num_dof, len(velocities)))

        if self.simulated:
            self.GetRobot().GetController().Reset(0)
            self.servo_simulator.SetVelocity(velocities)
        else:
            raise NotImplementedError('Servo is not implemented.') 
            
    
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
        @param ignore_collisions collisions with these objects are ignored when 
        planning the path, e.g. the object you think you will touch
        @param velocity_limit_scale A multiplier to use to scale velocity limits 
        when executing MoveUntilTouch ( < 1 in most cases).           
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


        # from IPython import embed
        # embed()

        with env:
            # Compute the expected force direction in the hand frame.
            hand_pose = manipulator.GetEndEffectorTransform()

            '''
            hand_pose[0:3, 0:3] = robot transformation relative to the world
            direction = force direction relative to the world
            force_direction = force direction relative to the robot
            so
            force_direction = direction * hand_pose[0:3, 0:3]
            '''
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            '''
            hand_pose
            [[ -5.82797374e-01   5.48233539e-01   5.99822646e-01  -4.39829960e-02]
             [  3.81656988e-01   8.36325169e-01  -3.93571030e-01   4.08090209e-01]
             [ -7.17415615e-01  -4.45658718e-04  -6.96645274e-01   2.14487209e-01]
             [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
            hand_pose[0:3, 0:3].T
            [[ -5.82797374e-01   3.81656988e-01  -7.17415615e-01]
             [  5.48233539e-01   8.36325169e-01  -4.45658718e-04]
             [  5.99822646e-01  -3.93571030e-01  -6.96645274e-01]]
            direction
            [ 0.  0. -1.]
            -direction
            [-0. -0.  1.]
            force_direction
            [ -7.17415615e-01  -4.45658718e-04  -6.96645274e-01]
            '''

            # Disable the KinBodies listed in ignore_collisions. We backup the
            # "enabled" state of all KinBodies so we can restore them later.
            body_savers = [body.CreateKinBodyStateSaver() for body in ignore_collisions]
            robot_saver = robot.CreateRobotStateSaver(
                  Robot.SaveParameters.ActiveDOF
                | Robot.SaveParameters.ActiveManipulator
                | Robot.SaveParameters.LinkTransformation)
            
            # h = openravepy.misc.DrawAxes(env,manipulator.GetEndEffectorTransform())

            with robot_saver, nested(*body_savers) as f:
                manipulator.SetActive()
                robot_cspec = robot.GetActiveConfigurationSpecification()

                for body in ignore_collisions:
                    body.Enable(False)

                # print '1\n'
                # print robot.arm.GetEndEffectorTransform();
                '''
                This situation always fails
                [[ -3.88388813e-03   9.63989264e-01  -2.65912796e-01   3.01630667e-01]
                 [  1.40669606e-02   2.65941158e-01   9.63886623e-01   2.14262567e-01]
                 [  9.99893512e-01   3.04299086e-06  -1.45932847e-02   2.14495531e-01]
                 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
                '''
                # print robot.arm.GetArmDOFValues();
                # print direction = [0., 0., -1.]
                # @param direction unit vector for the direction of motion in the world frame
                # print distance = 0.1
                # print max_distance = 0.15
                # plan to move a certain distance to the direction
                # direction should be [0,0,-1] because it is the direction relative to the world
                
                # I. We can plan this move downward motion
                # current_pose = robot.arm.GetEndEffectorTransform()
                # print current_pose
                # current_pose[2,3] = current_pose[2,3] - distance
                # print current_pose
                # path = robot.PlanToEndEffectorPose(current_pose)
                # print direction
                # print distance
                # print max_distance
                # II. We can just move it down
                path = robot.PlanToEndEffectorOffset(direction=direction, 
                    distance=distance, max_distance=max_distance, **kw_args)
                # print '2\n'
                # Here, if you try to robot.ExecutePath(path), then it will fail. 
                # Because the distance is too big. This is just for generating a path that long enough to reach the table
                # Then we will truncate this long path from the beginning to when it contacts with the table

        traj = robot.PostProcessPath(path)
        is_collision = False

        traj_cspec = traj.GetConfigurationSpecification()
        new_traj = RaveCreateTrajectory(env, '')
        new_traj.Init(traj_cspec)

        robot_saver = robot.CreateRobotStateSaver(
            Robot.SaveParameters.LinkTransformation)
        
        '''
        we separate the entire trajectory into small pieces
            so that we can move a little by discretely setting up the joint configurations
            and at the same time we will check if collision exists or not in each piece
        '''
        with env, robot_saver:
            # delta_t = 0.01 which was defined before
            for t in numpy.arange(0, traj.GetDuration(), delta_t):
                # sample a small step in this delta amount of time
                waypoint = traj.Sample(t)

                dof_values = robot_cspec.ExtractJointValues(
                    waypoint, robot, dof_indices, 0)
                manipulator.SetDOFValues(dof_values)

                # Terminate if we detect collision with the environment.
                report = CollisionReport()
                if env.CheckCollision(robot, report=report):
                    # logger.info('Terminated from collision: %s',
                    #     str(CollisionPlanningError.FromReport(report)))
                    print '--------------Terminated from collision\n'
                    is_collision = True
                    break
                elif robot.CheckSelfCollision(report=report):
                    # logger.info('Terminated from self-collision: %s',
                    #     str(CollisionPlanningError.FromReport(report)))
                    print '--------------Terminated from self-collision\n'
                    is_collision = True
                    break
                
                # Build the output trajectory that stops in contact.
                # GetNumWaypoints - return the number of waypoints
                # ???????????? why do we need traj_cspec
                if new_traj.GetNumWaypoints() == 0:
                    # InsertDeltaTime - sets the deltatime field of the data if one exists  
                    traj_cspec.InsertDeltaTime(waypoint, 0.)
                else:
                    traj_cspec.InsertDeltaTime(waypoint, delta_t)
                new_traj.Insert(new_traj.GetNumWaypoints(), waypoint)

        print 'GetNumWaypoints = ' + str(new_traj.GetNumWaypoints()) + '\n'
        if new_traj.GetNumWaypoints() > 0:
            robot.ExecuteTrajectory(new_traj)

        return is_collision
