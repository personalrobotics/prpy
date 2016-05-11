import logging

from ros_control_client_py import FollowJointTrajectoryClient, TrajectoryExecutionFailed
from ros_control_client_py.util import or_to_ros_trajectory


class OrController(object):
    """Dummy/empty OpenRAVE controller class"""

    def GetControlDOFIndices(self):
        raise NotImplementedError("GetControlDOFIndices not implemented")

    def GetNamespace(self):
        return self.namespace

    def GetRobot(self):
        return self.robot

    def Reset(self):
        raise NotImplementedError("Reset not implemented")

    def SetDesired(self, positions):
        raise NotImplementedError("SetDesired not implemented")

    def SetPath(self, traj):
        raise NotImplementedError("SetPath not implemented")

    def SimulationStep(self, dt):
        raise NotImplementedError("SimulatedStep not implemented")

    def IsDone(self):
        raise NotImplementedError("IsDone not implemented")

    def GetTime(self):
        raise NotImplementedError("GetTime not implemented")
    
    def GetVelocity(self):
        raise NotImplementedError("GetVelocity not implemented")

    def GetTorque(self):
        raise NotImplementedError("GetTorque not implemented")


class RewdOrController(OrController):
    """A super class for initializing all RewdOrControllers"""
    def __init__(self, robot, namespace, joint_names, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.robot = robot
        self.namespace = namespace
        self.joint_names = joint_names
        self.simulated = simulated
        self.logger.info("Rewd Controller initialized")


class RewdOrTrajectoryController(RewdOrController):
    """A controller for trajectory execution

    An interface for using trajectory controllers that
    is "compatible" with how we use OpenRAVE controllers
    """
    def __init__(self, robot, namespace, controller_name, joint_names,
                 simulated=False):
        super(RewdOrTrajectoryController, self).__init__(robot,
                                                         namespace,
                                                         joint_names,
                                                         simulated)
        if simulated:
            raise NotImplementedError('Simulation not supported in RewdOrTrajectoryController')

        self.controller_client = FollowJointTrajectoryClient(namespace,
                                                             controller_name)
        self.current_trajectory = None
        self.logger.info('Rewd Trajectory Controller initialized')

    def SetPath(self, traj):
        if not self.IsDone():
            raise TrajectoryExecutionFailed('Currently executing another trajectory', traj, None)

        ros_traj = or_to_ros_trajectory(self.GetRobot(), traj)
        self.current_trajectory = self.controller_client.execute(ros_traj)

    def IsDone(self):
        return self.current_trajectory is None or self.current_trajectory.done()

    def GetTime(self):
        # TODO implement with self.current_trajectory.partial_result()
        raise NotImplementedError('GetTime not yet implemented in RewdOrTrajectoryController')
