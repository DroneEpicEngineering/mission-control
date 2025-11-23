import numpy as np

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
    NavigationState,
)
from flight_control.navigation import NavigationStrategy
from flight_control.navigation.calculations import (
    calculate_distance,
    calculate_approach_velocity,
    calculate_los,
    calculate_los_change,
)


class ProportionalNavigation(NavigationStrategy):
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

        p_t = np.array([data.target_x, data.target_y, data.target_z])
        p_d = np.array([data.x, data.y, data.z])
        dp = p_t - p_d

        a_dir = dp / np.linalg.norm(dp)
        a_n = self._N * Vc * d_los * a_dir
        result = np.clip(a_n, -self._a_max, self._a_max)

        psi = data.psi + (a_n / self._Vd) * data.dt
        psi = np.arctan2(np.sin(psi), np.cos(psi))

        return NavigationOutput(ax=result[0], ay=result[1], az=result[2], psi=psi)
