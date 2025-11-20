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
    def __init__(self, **kwargs) -> None:
        super().__init__()
        self._state: FRPNState = None
        self._G = kwargs.get("G", 0)
        self._W = kwargs.get("W", 0)
        self._a_max = 5.0
        self._vd_max = 2.0

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
        p_t = np.array([data.target_x, data.target_y])
        p_d = np.array([data.x, data.y])

        v_d = (p_d - np.array([self._state.x, self._state.y])) / data.dt
        v_t = (p_t - np.array([self._state.target_x, self._state.target_y])) / data.dt

        dp = p_t - p_d
        dv = v_t - v_d

        dp_norm = np.linalg.norm(dp)
        dv_norm = np.linalg.norm(dv)

        eps = 1e-6
        tgo = dp_norm / (dv_norm + eps)
        tgo = np.clip(tgo, 0.05, 50.0)

        licznik = (1 - self._W) * dp + dv * tgo
        a_cmd = self._G * (licznik / (tgo**2) + self._W * dp)

        a_norm = np.linalg.norm(a_cmd)
        if a_norm > self._a_max:
            a_cmd = a_cmd * (self._a_max / a_norm)

        v_d = v_d + a_cmd * data.dt

        v_norm = np.linalg.norm(v_d)
        if v_norm > self._vd_max:
            v_d = v_d * (self._vd_max / v_norm)

        p_d = p_d + v_d * data.dt

        x = p_d[0]
        y = p_d[1]
        z = data.z

        self._state.target_x = data.target_x
        self._state.target_y = data.target_y
        self._state.target_z = data.target_z
        self._state.x = data.x
        self._state.y = data.y
        self._state.z = data.z

        return NavigationOutput(x=x, y=y, z=z, psi=data.psi)
