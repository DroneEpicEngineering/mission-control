from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl


class HeightCheck(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._target_height = 5.0
        self._offboard_control: OffboardControl = None
        self._position_xy: tuple[float, float] = (0.0, 0.0)

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def initialise(self) -> None:
        try:
            x, y, _ = self._offboard_control.local_position
        except ValueError:
            x, y = (0, 0)
        self._position_xy = (x, y)

    def update(self) -> Status:
        if self._offboard_control.is_position_reached(
            self._position_xy[0], self._position_xy[1], self._target_height
        ):
            return Status.SUCCESS

        return Status.FAILURE
