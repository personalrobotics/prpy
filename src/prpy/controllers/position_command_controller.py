import logging

from . import OrController
from ros_control_client_py import SetPositionClient, SetPositionFailed


class PositionCommandController(OrController):
    def __init__(self, namespace, controller_name, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self.simulated = simulated
        self.controller_client = SetPositionClient(namespace, controller_name)
        self._current_cmd = None
        self.logger.info("Position Command Controller initialized")

    def SetPosition(self, position):
        if not self.IsDone():
            raise SetPositionFailed("PositionCommand action already \
                                     in progress and cannot be preempted",
                                    position, None)

        self._current_cmd = self.controller_client.execute(position)

    def IsDone(self):
        return self._current_cmd is not None and self._current_cmd.done()
