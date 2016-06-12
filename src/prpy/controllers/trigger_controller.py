import logging

from . import OrController


class TriggerController(OrController):
    """A controller for triggering actions

    An interface for using trigger controller
    is "compatible" with how we use OpenRAVE controllers
    """
    def __init__(self, namespace, controller_name, simulated=False):
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self._current_cmd = None
        self.simulated = simulated
        if not simulated:
            from ros_control_client_py import TriggerClient
            self.controller_client = TriggerClient(namespace, controller_name)
        self.logger.info("Trigger Controller initialized")

    def Trigger(self, timeout=None):
        """Tigger the controlled action

        :param timeout: if not None, block until timeout
        :type timeout: double or None
        """
        if self.simulated:
            pass

        if not self.IsDone():
            from ros_control_client_py import TriggerFailed
            raise TriggerFailed("Trigger action already \
                                     in progress and cannot be preempted",
                                None)

        if not self.simulated:
            self._current_cmd = self.controller_client.execute()
            if timeout is not None and timeout >= 0.0:
                self._current_cmd.result(timeout)

    def IsDone(self):
        return (self.simulated or
                self._current_cmd is None or
                self._current_cmd.done())
