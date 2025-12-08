from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl

class ArmCheck(Behaviour):
    """Check for UAV arm."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None
    
    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()
    
    def update(self) -> Status:
        if self._offboard_control.is_armed:
            return Status.SUCCESS

        return Status.FAILURE
