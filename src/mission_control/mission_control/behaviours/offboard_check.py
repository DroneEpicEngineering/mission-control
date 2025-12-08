from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class OffboardCheck(Behaviour):
    """Check for UAV offboard flight mode."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def update(self) -> Status:
        if self._offboard_control.is_in_offboard and self._offboard_control.is_ready:
            return Status.SUCCESS

        return Status.FAILURE
