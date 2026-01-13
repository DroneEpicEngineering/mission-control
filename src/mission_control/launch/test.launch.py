from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_path("mission_control")

    params_file = LaunchConfiguration("params_file")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=(package_share / "params" / "example_params.yaml").as_posix(),
    )

    microxrce = ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"])

    behaviour_tree = Node(
        executable="behaviour_tree",
        package="mission_control",
        parameters=[params_file],
    )

    logger = Node(
        executable="position_logger", package="position_logger", output="screen"
    )

    autostart = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/start",
            "std_msgs/msg/Empty",
            "{}",
            "--once",
        ]
    )

    ld = LaunchDescription()

    ld.add_action(params_file_arg)

    ld.add_action(microxrce)
    ld.add_action(behaviour_tree)
    ld.add_action(logger)
    ld.add_action(autostart)

    return ld
