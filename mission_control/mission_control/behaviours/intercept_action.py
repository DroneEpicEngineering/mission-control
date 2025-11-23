import time

import numpy as np

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from flight_control.offboard_control_node import OffboardControl
from flight_control.navigation import NavigationStrategy, NavigationContext
from flight_control.navigation.types import NavigationInput, NavigationOutput


class InterceptAction(Behaviour):
    def __init__(self, name: str, strategy: NavigationStrategy) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None
        self._context: NavigationContext = None
        self._strategy = strategy
        self._blackboard = self.attach_blackboard_client("intercept_action")
        self._blackboard.register_key("target", access=Access.READ)
        self._prev_time = time.perf_counter()

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()
        self._context = NavigationContext(self._strategy)

    def update(self) -> Status:
        target_data = self._blackboard.get("target")
        local_position = np.array(self._offboard_control.local_position)

        now = time.perf_counter()

        data = NavigationInput(
            target_x=target_data.pose.position.x,
            target_y=target_data.pose.position.y,
            target_z=target_data.pose.position.z,
            x=local_position[0],
            y=local_position[1],
            z=local_position[2],
            psi=self._offboard_control.heading,
            dt=now - self._prev_time,
        )
        result = self._context.execute(data)

        self._offboard_control.get_logger().warn(f"\n{data}\n{result}\n")

        self._offboard_control.fly_acceleration(
            result.ax,
            result.ay,
            result.az,
            yaw=result.psi,
        )

        self._prev_time = time.perf_counter()

        return Status.RUNNING
