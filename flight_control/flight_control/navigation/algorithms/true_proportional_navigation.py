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
    def __init__(self, **kwargs) -> None:
        super().__init__()
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

        vx = self._Vd * np.cos(data.psi) + ax * data.dt
        vy = self._Vd * np.sin(data.psi) + ay * data.dt

        x = data.x + vx * data.dt
        y = data.y + vy * data.dt
        z = data.z
        psi = np.arctan2(vy, vx)

        self._state.los = calculate_los(data)
        self._state.R = calculate_distance(data)

        return NavigationOutput(x=x, y=y, z=z, psi=psi)
