from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    behaviour_tree = Node(
        executable="behaviour_tree",
        package="mission_control",
        parameters=[{"foo": "baz"}],
    )

    ld = LaunchDescription()

    ld.add_action(behaviour_tree)

    return ld
