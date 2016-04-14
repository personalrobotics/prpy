import logging

from . import OrController


class ForceTorqueSensorController(OrController):
    def __init__(self, namespace, controller_name, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self.simulated = simulated
        self.controller_client = ForceTorqueSensorController(namespace,
                                                             controller_name)
        self.logger.info("Force/Torque Sensor Controller initialized")

    def GetForceTorque(self):
        if not self.IsDone():
            pass  # TODO raise

        self._current_cmd = self.controller_client.execute()

    def IsDone(self):
        return self._current_cmd is not None and self._current_cmd.done()
