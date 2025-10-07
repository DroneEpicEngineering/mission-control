import rclpy
from rclpy.node import Node


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__("offboard_control")


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
