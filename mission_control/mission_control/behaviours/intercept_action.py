from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl
from flight_control.navigation import NavigationStrategy, NavigationContext

class InterceptAction(Behaviour):
    def __init__(self, name: str, strategy: NavigationStrategy) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None
        self._strategy = strategy

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()
    
    def update(self) -> Status:
        return Status.RUNNING