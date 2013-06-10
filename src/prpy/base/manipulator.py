import numpy, openravepy
from .. import clone, planning

class Manipulator(openravepy.Robot.Manipulator):
    def __init__(self):
        planning.Planner.bind(self, lambda: self.planner, executer=self._PlanWrapper)

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
        from clone import *
        robot = manipulator.GetRobot()
        with Clone(robot.GetEnv()):
            Cloned(robot).SetActive()
            return Cloned(robot)._PlanWrapper(planning_method, args, kw_args)
