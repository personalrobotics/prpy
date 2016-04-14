import logging

from . import OrController
from ros_control_client_py import TriggerClient, TriggerFailed


class TriggerController(OrController):
    def __init__(self, namespace, controller_name, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self.simulated = simulated
        self.controller_client = TriggerClient(namespace, controller_name)
        self.logger.info("Trigger Controller initialized")

    def Trigger(self):
        if not self.IsDone():
            raise TriggerFailed("Trigger action already \
                                     in progress and cannot be preempted",
                                None)

        self._current_cmd = self.controller_client.execute()

    def IsDone(self):
        return self._current_cmd is not None and self._current_cmd.done()
