import numpy as np

from flight_control.navigation import NavigationStrategy
from flight_control.navigation.types import NavigationInput, NavigationOutput


class FastResponseProportionalNavigation(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self._state = None
        self._G = kwargs.get("G", 0)
        self._W = kwargs.get("W", 0)

    def setup(self, data: NavigationInput) -> None:
        pass

    def execute(self, data: NavigationInput) -> NavigationOutput:
        uav_position = np.array(data.uav_odom.position)
        uav_velocity = np.array(data.uav_odom.velocity)
        target_position = np.array(data.target_odom.position)
        target_velocity = np.array(data.target_odom.velocity)

        relative_position = target_position - uav_position
        relative_velocity = target_velocity - uav_velocity

        position_norm = np.linalg.norm(relative_position)
        velocity_norm = np.linalg.norm(relative_velocity)

        eps = 1e-6
        tgo = position_norm / (velocity_norm + eps)
        tgo = np.clip(tgo, 0.05, 50.0)

        a_cmd = self._G * (
            (1 - self._W) * (relative_position + relative_velocity * tgo) / (tgo**2)
            + self._W * relative_position
        )
        result = np.clip(a_cmd, -self._a_max, self._a_max)

        return NavigationOutput(ax=result[0], ay=result[1], az=result[2], psi=data.psi)
