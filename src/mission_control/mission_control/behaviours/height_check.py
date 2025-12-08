from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl
from flight_control.coordinate_transforms import SpatialVector


class HeightCheck(Behaviour):
    def __init__(self, name: str, height: float) -> None:
        super().__init__(name)
        self._target_height = height
        self._offboard_control: OffboardControl = None
        self._current_position = SpatialVector.from_origin()

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def initialise(self) -> None:
        try:
            self._current_position = self._offboard_control.local_position
        except ValueError:
            self._current_position = SpatialVector.from_origin()

    def update(self) -> Status:
        target_coords = self._current_position.as_enu_coords()
        target_coords.z = self._target_height
        if self._offboard_control.is_position_reached(
            SpatialVector.from_enu_coords(target_coords)
        ):
            return Status.SUCCESS

        return Status.FAILURE
