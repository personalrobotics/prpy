import logging

from ros_control_client_py import FollowJointTrajectoryClient, TrajectoryExecutionFailed
from ros_control_client_py.util import or_to_ros_trajectory


class OrController(object):
    """Dummy/empty OpenRAVE controller class"""

    def GetControlDOFIndices(self):
        pass

    def GetNamespace(self):
        return self.namespace

    def GetRobot(self):
        return self.robot

    def Reset(self):
        pass

    def SetDesired(self, positions):
        pass

    def SetPath(self, traj):
        pass

    def SimulationStep(self, dt):
        pass

    def IsDone(self):
        pass

    def GetTime(self):
        pass
    
    def GetVelocity(self):
        pass

    def GetTorque(self):
        pass


class RewdOrController(OrController):
    def __init__(self, robot, namespace, joint_names, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.robot = robot
        self.namespace = namespace
        self.joint_names = joint_names
        self.simulated = simulated
        self.logger.info("Rewd Controller initialized")


class RewdOrGravityCompensationController(RewdOrController):
    def __init__(self, robot, namespace, joint_names, simulated=False):
        super(RewdOrGravityCompensationController, self).__init__(robot,
                                                                  namespace,
                                                                  joint_names,
                                                                  simulated)

        self.logger.info("Rewd Gravity Compensation Controller initialized")


    def GetTorque(self):
        pass


class RewdOrTrajectoryController(RewdOrController):
    def __init__(self, robot, namespace, joint_names, simulated=False):
        super(RewdOrTrajectoryController, self).__init__(robot,
                                                         namespace,
                                                         joint_names,
                                                         simulated)

        self.controller_client = FollowJointTrajectoryClient(namespace + "/rewd_trajectory_controller")
        self.current_trajectory = None
        self.logger.info("Rewd Trajectory Controller initialized")

    def SetPath(self, traj):
        if not self.IsDone():
            raise TrajectoryExecutionFailed("Currently executing another trajectory", traj, None)

        ros_traj = or_to_ros_trajectory(self.GetRobot(), traj)
        self.current_trajectory = self.controller_client.execute(ros_traj)

    def IsDone(self):
        return self.current_trajectory is not None and self.current_trajectory.done()

    def GetTime(self):
        # TODO implement with self.current_trajectory.partial_result()
        pass
