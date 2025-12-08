from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class ReturnAction(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None
    
    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def update(self) -> Status:
        self._offboard_control.return_to_launch()

        return Status.RUNNING
