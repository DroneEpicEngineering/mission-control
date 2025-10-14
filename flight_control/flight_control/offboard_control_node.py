import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint

from utils.qos_profiles import PX4_PROFILE

from flight_control.coordinate_transforms import enu_to_ned


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

        self._heartbeat_timer = self.create_timer(0.2, self.__heartbeat_timer_cb)
        self._offboard_setpoint_counter = 0

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", PX4_PROFILE
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", PX4_PROFILE
        )
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", PX4_PROFILE
        )

    @property
    def is_ready(self) -> bool:
        return self._offboard_setpoint_counter > self.OFFBOARD_SETPOINT_THRESHOLD

    def arm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
        )

    def disarm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0
        )

    def land(self) -> None:
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_launch(self) -> None:
        self.__publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def set_offboard_mode(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def set_hold_mode(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0
        )

    def fly_point(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = enu_to_ned(x, y, z)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

    def fly_velocity(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = None
        msg.velocity = enu_to_ned(x, y, z)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

    def fly_acceleration(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = None
        msg.velocity = None
        msg.acceleration = enu_to_ned(x, y, z)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

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

    def __publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params["param1"] or 0
        msg.param2 = params["param2"] or 0
        msg.param3 = params["param3"] or 0
        msg.param4 = params["param4"] or 0
        msg.param5 = params["param5"] or 0
        msg.param6 = params["param6"] or 0
        msg.param7 = params["param7"] or 0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.__px4_timestamp_now()
        self._vehicle_command_pub.publish(msg)

    def __px4_timestamp_now(self) -> int:
        return self.get_clock().now().nanoseconds / 1000


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
