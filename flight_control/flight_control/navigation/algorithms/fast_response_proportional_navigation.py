from dataclasses import dataclass

import numpy as np

from flight_control.navigation import NavigationStrategy
from flight_control.navigation.types import NavigationInput, NavigationOutput


@dataclass
class FRPNState:
    target_x: float
    target_y: float
    target_z: float

    x: float
    y: float
    z: float


class FastResponseProportionalNavigation(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self._state: FRPNState = None
        self._G = kwargs.get("G", 0)
        self._W = kwargs.get("W", 0)

    def setup(self, data: NavigationInput) -> None:
        self._state = FRPNState(
            target_x=data.target_x,
            target_y=data.target_y,
            target_z=data.target_z,
            x=data.x,
            y=data.y,
            z=data.z,
        )

    def execute(self, data: NavigationInput) -> NavigationOutput:
        p_t = np.array([data.target_x, data.target_y, data.target_z])
        p_d = np.array([data.x, data.y, data.z])

        v_d = (p_d - np.array([self._state.x, self._state.y, self._state.z])) / data.dt
        v_t = (
            p_t
            - np.array(
                [self._state.target_x, self._state.target_y, self._state.target_z]
            )
        ) / data.dt

        dp = p_t - p_d
        dv = v_t - v_d

        dp_norm = np.linalg.norm(dp)
        dv_norm = np.linalg.norm(dv)

        eps = 1e-6
        tgo = dp_norm / (dv_norm + eps)
        tgo = np.clip(tgo, 0.05, 50.0)

        a_cmd = self._G * ((1 - self._W) * (dp + dv * tgo) / (tgo**2) + self._W * dp)
        result = np.clip(a_cmd, -self._a_max, self._a_max)

        return NavigationOutput(ax=result[0], ay=result[1], az=result[2], psi=data.psi)
