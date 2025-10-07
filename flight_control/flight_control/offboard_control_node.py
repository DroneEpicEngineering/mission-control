import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
)

from utils.qos_profiles import PX4_PROFILE


class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class OffboardControl(Node, metaclass=SingletonMeta):
    OFFBOARD_SETPOINT_THRESHOLD = 10

    def __init__(self) -> None:
        super().__init__("offboard_control")

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", PX4_PROFILE
        )

        self._heartbeat_timer = self.create_timer(0.2, self.__heartbeat_timer_cb)
        self._offboard_setpoint_counter = 0

    @property
    def is_ready(self) -> bool:
        return self._offboard_setpoint_counter > self.OFFBOARD_SETPOINT_THRESHOLD

    def __heartbeat_timer_cb(self) -> None:
        self.__publish_offboard_control_mode()
        if self._offboard_setpoint_counter <= self.OFFBOARD_SETPOINT_THRESHOLD:
            self._offboard_setpoint_counter += 1

    def __publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self._offboard_control_mode_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = OffboardControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
