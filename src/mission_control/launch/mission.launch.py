from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_path("mission_control")

    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=(package_share / "params" / "example_params.yaml").as_posix(),
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=(package_share / "config" / "visualization.rviz").as_posix(),
    )

    microxrce = ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"])
    tree_viewer = ExecuteProcess(cmd=["py-trees-tree-viewer"])

    behaviour_tree = Node(
        executable="behaviour_tree",
        package="mission_control",
        parameters=[params_file],
    )

    visualizer = Node(
        executable="visualizer",
        package="mission_control",
        parameters=[params_file],
    )

    rviz = Node(executable="rviz2", package="rviz2", arguments=["-d", rviz_config])

    logger = Node(
        executable="position_logger", package="position_logger", output="screen"
    )

    bag = ExecuteProcess(cmd=["ros2", "bag", "record", "-a"])

    ld = LaunchDescription()

    ld.add_action(params_file_arg)
    ld.add_action(rviz_config_arg)

    ld.add_action(microxrce)
    ld.add_action(tree_viewer)
    ld.add_action(behaviour_tree)
    ld.add_action(visualizer)
    ld.add_action(rviz)
    ld.add_action(logger)
    ld.add_action(bag)

    return ld
