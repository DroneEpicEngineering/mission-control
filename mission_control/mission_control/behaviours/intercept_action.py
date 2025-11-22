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

    def setup(self, **kwargs) -> None:
        self._offboard_control = OffboardControl()
        self._context = NavigationContext(self._strategy)

    def update(self) -> Status:
        target_data = self._blackboard.get("target")
        local_position = np.array(self._offboard_control.local_position)

        # if self._offboard_control.is_position_reached(
        #     target_data.pose.position.x,
        #     target_data.pose.position.y,
        #     target_data.pose.position.z,
        #     epsilon=1.0,
        # ):
        #     return Status.SUCCESS

        data = NavigationInput(
            target_x=target_data.pose.position.x,
            target_y=target_data.pose.position.y,
            target_z=target_data.pose.position.z,
            x=local_position[0],
            y=local_position[1],
            z=local_position[2],
            psi=self._offboard_control.heading,
            dt=0.01,
        )
        result = self._context.execute(data)

        self._offboard_control.fly_point(
            result.x,
            result.y,
            result.z,
            result.psi,
        )

        self._offboard_control.get_logger().warn(f"\n{data}\n{result}\n")

        # next_position = np.array([result.x, result.y, result.z])
        # step_vector = (next_position - local_position) / np.norm(
        #     next_position - local_position
        # )

        # self._offboard_control.fly_velocity(
        #     step_vector[0],
        #     step_vector[1],
        #     step_vector[2],
        #     result.psi,
        # )

        return Status.RUNNING
