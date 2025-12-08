import time
import numpy as np

from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from flight_control.offboard_control_node import OffboardControl
from flight_control.coordinate_transforms import SpatialVector
from flight_control.navigation import NavigationStrategy, NavigationContext
from flight_control.navigation.types import NavigationInput, Odometry
from flight_control.navigation import algorithms as algs


class InterceptAction(Behaviour):
    def __init__(
        self,
        name: str,
        strategy: NavigationStrategy,
        mission_threshold: int = 50,
        distance_threshold: float = 3.0,
    ) -> None:
        super().__init__(name)
        self._offboard_control: OffboardControl = None
        self._context: NavigationContext = None
        self._strategy = strategy
        self._blackboard = self.attach_blackboard_client("intercept_action")
        self._blackboard.register_key("target", access=Access.READ)
        self._blackboard.register_key("finish", access=Access.WRITE)
        self._prev_time = time.perf_counter()
        self._pure_pursuit = algs.PurePursuit()
        self._prev_timestamp = None

        self._mission_threshold = mission_threshold
        self._distance_threshold = distance_threshold

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
            *self._offboard_control.local_position.as_enu(),
            *self._offboard_control.velocity.as_enu(),
            psi=self._offboard_control.heading,
        )

        if (
            uav_odom.position[0] >= self._mission_threshold
            or uav_odom.position[1] >= self._mission_threshold
        ):
            self._blackboard.set("finish", value=True)
            self._offboard_control.get_logger().error(
                "Out of mission boundary, returning."
            )
            return Status.FAILURE

        if np.linalg.norm(
            np.array(target_odom.position) - np.array(uav_odom.position)
        ) < 1.0 and type(self._strategy) in (
            algs.ProportionalNavigation,
            algs.TrueProportionalNavigation,
        ):
            self._offboard_control.get_logger().warn("changing to Pure Pursuit")
            self._context.strategy = self._pure_pursuit

        if (
            np.linalg.norm(np.array(target_odom.position) - np.array(uav_odom.position))
            < self._distance_threshold
        ):
            self._offboard_control.get_logger().info("Stopping to avoid collision.")
            self._offboard_control.fly_acceleration(SpatialVector.from_origin())

            self._prev_time = time.perf_counter()
            return Status.RUNNING

        data = NavigationInput(target_odom, uav_odom, dt=now - self._prev_time)
        result = self._context.execute(data)

        self._offboard_control.fly_acceleration(
            SpatialVector.from_enu(result.ax, result.ay, result.az),
            yaw=result.psi,
        )

        self._prev_time = time.perf_counter()

        return Status.RUNNING
