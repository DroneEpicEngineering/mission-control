#!/usr/bin/env python3
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from system_interfaces.action._follow_trajectory import FollowTrajectory_FeedbackMessage


class PositionLogger(Node):
    def __init__(self):
        super().__init__("position_logger")
        self.drone_pos = None
        self.target_pos = None

        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(os.getcwd(), f"positions_log_{timestamp}.csv")
        with open(self.csv_file, "w", newline="") as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(
                [
                    "sim_time",
                    "drone_x",
                    "drone_y",
                    "drone_z",
                    "target_x",
                    "target_y",
                    "target_z",
                ]
            )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.drone_callback,
            px4_qos,
        )

        self.create_subscription(
            FollowTrajectory_FeedbackMessage,
            "/follow_trajectory/_action/feedback",
            self.target_callback,
            1,
        )

        self.create_timer(0.1, self.log_positions)

    def drone_callback(self, msg: VehicleLocalPosition):
        if self.drone_pos is None:
            self.get_logger().warn("drone_pos initialized")

        self.drone_pos = msg

    def target_callback(self, msg: FollowTrajectory_FeedbackMessage):
        if self.target_pos is None:
            self.get_logger().warn("target_pos initialized")

        self.target_pos = msg.feedback.pose.position

    def log_positions(self):
        if self.drone_pos is None or self.target_pos is None:
            return

        sim_time = self.get_clock().now().nanoseconds * 1e-9

        with open(self.csv_file, "a", newline="") as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(
                [
                    sim_time,
                    self.drone_pos.x,
                    self.drone_pos.y,
                    self.drone_pos.z,
                    self.target_pos.x,
                    self.target_pos.y,
                    self.target_pos.z,
                ]
            )


def main(args=None):
    rclpy.init(args=args)
    node = PositionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
