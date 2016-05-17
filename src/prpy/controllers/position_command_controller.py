import logging
from . import OrController


class PositionCommandController(OrController):
    """A controller for stateful position control

    An interface to using the position_command_controller package
    that is "compatible" with how we use OpenRAVE controllers
    """
    def __init__(self, namespace, controller_name, simulated=False,
                 connection_timeout=10.0):
        if simulated:
            raise NotImplementedError('Simulation not supported on '
                                      'PositionCommandController')
        from ros_control_client_py import SetPositionClient
        self.logger = logging.getLogger(__name__)
        self.namespace = namespace
        self.controller_name = controller_name
        self.controller_client = SetPositionClient(namespace, controller_name,
                                                   connection_timeout)
        self._current_cmd = None
        self.logger.info('Position Command Controller {}/{} initialized'
                         .format(namespace, controller_name))

    def SetDesired(self, position):
        from ros_control_client_py import SetPositionFailed
        if not self.IsDone():
            raise SetPositionFailed('PositionCommand action already '
                                    'in progress and cannot be preempted',
                                    position, None)

        self.logger.info('Sending position: {}'.format(position))
        self._current_cmd = self.controller_client.execute(position)

    def IsDone(self):
        return self._current_cmd is None or self._current_cmd.done()
