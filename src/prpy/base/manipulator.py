import copy, numpy, openravepy
from .. import clone, planning

class Manipulator(openravepy.Robot.Manipulator):
    def __init__(self):
        planning.Planner.bind(self, lambda: self.GetRobot().planner, executer=self._PlanWrapper)

    def CloneBindings(self, parent):
        self.__init__(self)

    def GetIndices(self):
        return self.GetArmIndices()

    def GetDOFValues(self):
        return self.GetRobot().GetDOFValues(self.GetIndices())

    def SetDOFValues(self, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        self.GetRobot().SetDOFValues(dof_values, self.GetIndices(), limits_action)

    def SetActive(self):
        self.GetRobot().SetActiveManipulator(self)
        self.GetRobot().SetActiveDOFs(self.GetArmIndices())

    def SetVelocityLimits(self, velocity_limits, min_accel_time):
        velocity_limits = numpy.array(velocity_limits, dtype='float')
        active_indices = self.GetIndices()

        # Set the velocity limits.
        or_velocity_limits = self.GetRobot().GetDOFVelocityLimits()
        or_velocity_limits[active_indices] = velocity_limits
        self.GetRobot().SetDOFVelocityLimits(or_velocity_limits)

        # Set the acceleration limits.
        or_accel_limits = self.GetRobot().GetDOFAccelerationLimits()
        or_accel_limits[active_indices] = velocity_limits / min_accel_time
        self.GetRobot().SetDOFAccelerationLimits(or_accel_limits)

    def _PlanWrapper(self, planning_method, args, kw_args):
        from prpy.clone import *
        robot = self.GetRobot()
        with Clone(robot.GetEnv()):
            Cloned(self).SetActive()
            cloned_args = copy.copy(kw_args)
            cloned_args['execute'] = False
            cloned_traj = Cloned(robot)._PlanWrapper(planning_method, args, cloned_args)

            # Strip inactive DOFs from the trajectory.
            config_spec = Cloned(robot).GetActiveConfigurationSpecification()
            openravepy.planningutils.ConvertTrajectorySpecification(cloned_traj, config_spec)
            traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
            traj.Clone(cloned_traj, 0)

        # Optionally execute the trajectory.
        if 'execute' not in kw_args or kw_args['execute']:
            return self.GetRobot().ExecuteTrajectory(traj, **kw_args)
        else:
            return traj
