import numpy as np

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
    NavigationState,
)
from flight_control.navigation import NavigationStrategy
from flight_control.navigation.calculations import (
    calculate_los,
    calculate_los_change,
    calculate_distance,
    calculate_approach_velocity,
)


class TrueProportionalNavigation(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self._state: NavigationState = None
        self._N = kwargs.get("N", 4)
        self._Vd = kwargs.get("Vd", 2)

    def setup(self, data: NavigationInput) -> None:
        self._state = NavigationState(
            los=calculate_los(data),
            R=calculate_distance(data),
        )

    def execute(self, data: NavigationInput) -> NavigationOutput:
        los = calculate_los(data)
        d_los = calculate_los_change(los, self._state.los, data.dt)
        R = calculate_distance(data)
        Vc = calculate_approach_velocity(R, self._state.R, data.dt)

        a_n = self._N * Vc * d_los

        n_los_x = -np.sin(los)
        n_los_y = np.cos(los)

        ax = a_n * n_los_x
        ay = a_n * n_los_y

        uav_position = np.array(data.uav_odom.position)
        target_position = np.array(data.target_odom.position)

        relative_position = target_position - uav_position

        a_dir = relative_position / np.linalg.norm(relative_position)
        a_cmd = np.array([ax * a_dir[0], ay * a_dir[1], 1 * a_dir[2]])
        result = np.clip(a_cmd, -self._a_max, self._a_max)

        return NavigationOutput(ax=result[0], ay=result[1], az=result[2], psi=data.psi)
