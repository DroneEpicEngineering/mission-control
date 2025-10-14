from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class WaitForConnection(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def update(self) -> Status:
        try:
            self._offboard_control.is_in_offboard
        except ValueError:
            return Status.RUNNING

        return Status.SUCCESS
