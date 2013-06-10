import openravepy

class EndEffector(openravepy.Robot.Link):
    def __init__(self, manipulator):
        self.manipulator = manipulator

    def GetIndices(self):
        indices = self.manipulator.GetChildDOFIndices()
        indices.sort()
        return indices

    def GetDOFValues(self):
        return self.manipulator.GetRobot().GetDOFValues(self.GetIndices())

    def SetDOFValues(self, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        hand.robot.SetDOFValues(dof_values, hand.GetIndices(), limits_action)

    def SetActive(self):
        self.GetRobot().SetActiveManipulator(self.manipulator)
        self.GetRobot().SetActiveDOFs(self.GetArmIndices())
