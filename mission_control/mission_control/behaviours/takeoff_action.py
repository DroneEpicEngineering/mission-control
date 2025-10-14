from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class TakeoffAction(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._target_height = 5.0
        self._offboard_control: OffboardControl = None
        self._position_xy = (0.0, 0.0)
        self._command_sent = False

    def setup(self) -> None:
        self._offboard_control = OffboardControl()

    def initialise(self) -> None:
        self._position_xy[0], self._position_xy[1], _ = (
            self._offboard_control.local_position
        )

    def update(self) -> Status:
        if not self._command_sent:
            self._offboard_control.fly_point(
                self._position_xy[0], self._position_xy[1], self._target_height
            )
            self._command_sent = True

        return Status.RUNNING
