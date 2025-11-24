import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from flight_control.offboard_control_node import OffboardControl
from flight_control.navigation import NavigationStrategy, NavigationContext
from flight_control.navigation.types import NavigationInput, Odometry


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
        target_position = target_data.pose.position
        target_twist = target_data.twist.linear

        now = time.perf_counter()

        target_odom = Odometry(
            target_position.x,
            target_position.y,
            target_position.z,
            target_twist.x,
            target_twist.y,
            target_twist.z,
            psi=0.0,
        )
        uav_odom = Odometry(
            *self._offboard_control.local_position,
            *self._offboard_control.velocity,
            psi=self._offboard_control.heading,
        )

        data = NavigationInput(target_odom, uav_odom, dt=now - self._prev_time)
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
