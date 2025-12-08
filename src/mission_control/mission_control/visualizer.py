import rclpy
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from px4_msgs.msg import VehicleLocalPosition
from system_interfaces.action._follow_trajectory import FollowTrajectory_FeedbackMessage

from utils.qos_profiles import PX4_PROFILE


class Visualizer(Node):
    def __init__(self) -> None:
        super().__init__("visualizer")
        self._uav_position_pub = self.create_publisher(
            PoseStamped, "vis/uav_position", 1
        )
        self._target_position_pub = self.create_publisher(
            PoseStamped, "vis/target_position", 1
        )
        self._vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.__vehicle_local_position_cb,
            PX4_PROFILE,
        )
        self._follow_trajectory_feedback_sub = self.create_subscription(
            FollowTrajectory_FeedbackMessage,
            "/follow_trajectory/_action/feedback",
            self.__follow_trajectory_feedback_cb,
            1,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

    def __vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        pose = PoseStamped()
        pose.pose.position.x = msg.y
        pose.pose.position.y = msg.x
        pose.pose.position.z = -msg.z
        pose.header = self.__generate_header()
        self._uav_position_pub.publish(pose)

        ts = TransformStamped()
        ts.header = self.__generate_header()
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = msg.y
        ts.transform.translation.y = msg.x
        ts.transform.translation.z = -msg.z
        self._tf_broadcaster.sendTransform(ts)

    def __follow_trajectory_feedback_cb(
        self, msg: FollowTrajectory_FeedbackMessage
    ) -> None:
        pose = PoseStamped()
        pose.pose = msg.feedback.pose
        pose.header = self.__generate_header()
        self._target_position_pub.publish(pose)

        ts = TransformStamped()
        ts.header = self.__generate_header()
        ts.child_frame_id = "target"
        ts.transform.translation.x = msg.feedback.pose.position.x
        ts.transform.translation.y = msg.feedback.pose.position.y
        ts.transform.translation.z = msg.feedback.pose.position.z
        self._tf_broadcaster.sendTransform(ts)

    def __generate_header(self) -> Header:
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()
        return header


def main():
    rclpy.init()
    visualizer = Visualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
