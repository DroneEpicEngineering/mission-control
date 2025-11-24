import numpy as np

from flight_control.navigation import NavigationStrategy
from flight_control.navigation.types import NavigationInput, NavigationOutput


class PurePursuit(NavigationStrategy):
    def __init__(self, a_max=1.0, **kwargs) -> None:
        super().__init__(a_max)
        self.lookahead_distance = kwargs.get("lookahead_distance", 5.0)
        self.kp = kwargs.get("kp", 2.0)

    def setup(self, data: NavigationInput) -> None:
        pass

    def execute(self, data: NavigationInput) -> NavigationOutput:
        uav_position = np.array(data.uav_odom.position)
        uav_velocity = np.array(data.uav_odom.velocity)
        target_position = np.array(data.target_odom.position)

        relative_position = target_position - uav_position

        distance_to_pursuit = np.linalg.norm(relative_position)

        desired_direction = relative_position / distance_to_pursuit
        desired_speed = min(np.linalg.norm(uav_velocity) + 1.0, self._a_max * 2.0)
        desired_velocity = desired_direction * desired_speed

        velocity_error = desired_velocity - uav_velocity
        a_cmd = self.kp * velocity_error

        a_cmd_norm = np.linalg.norm(a_cmd)
        if a_cmd_norm > self._a_max:
            a_cmd = a_cmd * (self._a_max / a_cmd_norm)

        psi = np.arctan2(relative_position[1], relative_position[0])

        return NavigationOutput(ax=a_cmd[0], ay=a_cmd[1], az=a_cmd[2], psi=psi)
