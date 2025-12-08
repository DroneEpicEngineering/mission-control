import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleLocalPosition,
)

from utils.qos_profiles import PX4_PROFILE

from flight_control.coordinate_transforms import (
    heading_transform,
    SpatialVector,
)


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

        self._vehicle_status: VehicleStatus = None
        self._vehicle_local_position: VehicleLocalPosition = None

        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status",
            self.__vehicle_status_cb,
            PX4_PROFILE,
        )
        self._vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.__vehicle_local_position_cb,
            PX4_PROFILE,
        )

    @property
    def is_ready(self) -> bool:
        return self._offboard_setpoint_counter > self.OFFBOARD_SETPOINT_THRESHOLD

    @property
    def is_in_offboard(self) -> bool:
        if self._vehicle_status is None:
            raise ValueError("Vehicle Status not initialized")

        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def is_armed(self) -> bool:
        if self._vehicle_status is None:
            raise ValueError("Vehicle Status not initialized")

        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def local_position(self) -> SpatialVector:
        if self._vehicle_local_position is None:
            raise ValueError("Vehicle Local Position not initialized")

        return SpatialVector.from_ned(
            self._vehicle_local_position.x,
            self._vehicle_local_position.y,
            self._vehicle_local_position.z,
        )

    @property
    def velocity(self) -> SpatialVector:
        if self._vehicle_local_position is None:
            raise ValueError("Vehicle Local Postion not initialized")

        return SpatialVector.from_ned(
            self._vehicle_local_position.vx,
            self._vehicle_local_position.vy,
            self._vehicle_local_position.vz,
        )

    @property
    def heading(self) -> float:
        if self._vehicle_local_position is None:
            raise ValueError("Vehicle Local Position not initialized")

        return self._vehicle_local_position.heading

    def arm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def disarm(self) -> None:
        self.__publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
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

    def fly_point(self, point: SpatialVector, yaw: float = None) -> None:
        msg = TrajectorySetpoint()
        msg.position = point.as_ned()
        msg.yaw = heading_transform(yaw if yaw is not None else self.heading)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

    def fly_velocity(self, velocity: SpatialVector, yaw: float = None) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = velocity.as_ned()
        msg.yaw = heading_transform(yaw if yaw is not None else self.heading)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

    def fly_acceleration(self, acceleration: SpatialVector, yaw: float = None) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [float("nan"), float("nan"), float("nan")]
        msg.acceleration = acceleration.as_ned()
        msg.yaw = heading_transform(yaw if yaw is not None else self.heading)
        msg.timestamp = self.__px4_timestamp_now()
        self._trajectory_setpoint_pub.publish(msg)

    def is_position_reached(self, target_position: SpatialVector, epsilon=0.1) -> bool:
        if self._vehicle_local_position is None:
            raise ValueError("Vehicle Local Position not initialized")

        position = (
            self._vehicle_local_position.x,
            self._vehicle_local_position.y,
            self._vehicle_local_position.z,
        )
        return all(
            abs(pos - target) <= epsilon
            for pos, target in zip(position, target_position.as_ned())
        )

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
        msg.timestamp = self.__px4_timestamp_now()
        self._offboard_control_mode_pub.publish(msg)

    def __publish_vehicle_command(self, command: int, **kwargs) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.__px4_timestamp_now()
        self._vehicle_command_pub.publish(msg)

    def __px4_timestamp_now(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def __vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def __vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self._vehicle_local_position = msg


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
