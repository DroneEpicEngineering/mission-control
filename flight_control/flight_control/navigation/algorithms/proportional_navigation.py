import numpy as np

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
)
from flight_control.navigation import NavigationStrategy


class ProportionalNavigation(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self._state = None
        self._N = kwargs.get("N", 4)

    def setup(self, data: NavigationInput) -> None:
        pass

    def execute(self, data: NavigationInput) -> NavigationOutput:
        uav_position = np.array(data.uav_odom.position)
        uav_velocity = np.array(data.uav_odom.velocity)

        target_position = np.array(data.target_odom.position)
        target_velocity = np.array(data.target_odom.velocity)

        relative_position = target_position - uav_position
        relative_velocity = target_velocity - uav_velocity
        distance = np.linalg.norm(relative_position)

        unit_position = relative_position / distance

        closing_velocity = -np.dot(unit_position, relative_velocity)

        dv_lateral = relative_velocity - np.dot(relative_velocity, unit_position) * unit_position
        u_dot = dv_lateral / distance

        lambda_dot = np.linalg.norm(u_dot)

        a_dir = u_dot / lambda_dot
        a_cmd = self._N * closing_velocity * lambda_dot * a_dir

        a_cmd = np.clip(a_cmd, -self._a_max, self._a_max)

        psi = data.uav_odom.psi + (a_cmd[0:2].dot([1, 1]) / 1.0) * data.dt
        psi = np.arctan2(np.sin(psi), np.cos(psi))

        return NavigationOutput(ax=a_cmd[0], ay=a_cmd[1], az=a_cmd[2], psi=psi)
