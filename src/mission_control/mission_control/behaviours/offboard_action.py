from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class OffboardAction(Behaviour):
    """Set UAV flight mode to offboard."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._command_sent = False
        self._offboard_control: OffboardControl = None

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def update(self) -> Status:
        if not self._command_sent:
            self._offboard_control.set_offboard_mode()
            self._command_sent = True

        return Status.RUNNING
