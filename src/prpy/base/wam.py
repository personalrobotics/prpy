import numpy, openravepy
from manipulator import Manipulator
from prpy.clone import Clone, Cloned
from .. import util

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
        with robot:
            robot.SetActiveManipulator(self)
            if iktype is not None:
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

    def MoveUntilTouch(manipulator, direction, distance, max_force=5, **kw_args):
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
        with manipulator.GetRobot().GetEnv():
            manipulator.GetRobot().GetController().SimulationStep(0)

            # Compute the expected force direction in the hand frame.
            direction = numpy.array(direction)
            hand_pose = manipulator.GetEndEffectorTransform()
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            with manipulator.GetRobot():
                traj = manipulator.PlanToEndEffectorOffset(direction, distance, execute=False, **kw_args)
                traj = manipulator.GetRobot().BlendTrajectory(traj)
                traj = manipulator.GetRobot().RetimeTrajectory(traj, stop_on_ft=True, force_direction=force_direction,
                                                           force_magnitude=max_force, torque=[100,100,100])

        # TODO: Use a simulated force/torque sensor in simulation.
        try:
            manipulator.hand.TareForceTorqueSensor()
            manipulator.GetRobot().ExecuteTrajectory(traj, execute=True, retime=False, blend=False)
            return False
        # Trajectory is aborted by OWD because we felt a force.
        except exceptions.TrajectoryAborted:
            return True
