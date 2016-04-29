import logging
import rospy
from geometry_msgs.msg import WrenchStamped
from . import OrController


class ForceTorqueSensorController(OrController):
    def __init__(self, namespace, controller_name, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self.simulated = simulated

        self.logger.info("Force/Torque Sensor Controller initialized")

    def GetForceTorque(self):
        if self.simulated:
            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            return msg
        else:
            if not self.IsDone():
                pass  # TODO raise
            else:
                return rospy.wait_for_message(self.controller_name,
                                              WrenchStamped,
                                              timeout=1.0)
                
    def IsDone(self):
        return 'done'
