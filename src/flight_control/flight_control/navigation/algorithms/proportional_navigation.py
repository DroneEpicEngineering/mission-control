import numpy as np

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
)
from flight_control.navigation import NavigationStrategy
from flight_control.navigation.calculations import (
    calculate_los,
    calculate_los_change,
)
from flight_control.navigation.types import NavigationState


class ProportionalNavigation(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self._state = None
        self._N = kwargs.get("N", 4)

    def setup(self, data: NavigationInput) -> None:
        self._state = NavigationState(los=calculate_los(data), R=0)

    def execute(self, data: NavigationInput) -> NavigationOutput:
        uav_position = np.array(data.uav_odom.position)
        uav_velocity = np.array(data.uav_odom.velocity)
        target_position = np.array(data.target_odom.position)
        target_velocity = np.array(data.target_odom.velocity)

        relative_position = target_position - uav_position
        relative_velocity = target_velocity - uav_velocity

        position_norm = np.linalg.norm(relative_position)
        velocity_norm = np.linalg.norm(relative_velocity)

        los = calculate_los(data)
        d_los = calculate_los_change(los, self._state.los, data.dt)

        a_dir = relative_position / position_norm
        a_n = self._N * velocity_norm * d_los * a_dir
        result = np.clip(a_n, -self._a_max, self._a_max)

        psi = np.arctan2(relative_position[1], relative_position[0])

        return NavigationOutput(ax=result[0], ay=result[1], az=result[2], psi=psi)
