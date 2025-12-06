from py_trees.behaviour import Behaviour
from py_trees.common import Status

from flight_control.offboard_control_node import OffboardControl
from flight_control.coordinate_transforms import SpatialVector


class TakeoffAction(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._target_height = 5.0
        self._offboard_control: OffboardControl = None
        self._current_position = SpatialVector.from_origin()
        self._command_sent = False

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()

    def initialise(self) -> None:
        self._current_position = self._offboard_control.local_position

    def update(self) -> Status:
        takeoff_coords = self._current_position.as_enu_coords()
        takeoff_coords.z = self._target_height
        self._offboard_control.fly_point(SpatialVector.from_enu_coords(takeoff_coords))

        return Status.RUNNING
