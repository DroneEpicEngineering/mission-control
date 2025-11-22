import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from py_trees.composites import Selector, Sequence, Composite
from py_trees.common import ClearingPolicy

from py_trees_ros.trees import BehaviourTree
from py_trees_ros.exceptions import TimedOutError
from py_trees_ros.subscribers import ToBlackboard, WaitForData

from std_msgs.msg import Empty

from flight_control.offboard_control_node import OffboardControl
from flight_control.navigation import NavigationStrategy, algorithms as algs
from system_interfaces.action._follow_trajectory import FollowTrajectory_FeedbackMessage

from mission_control import behaviours


def parameters() -> dict:
    param_reader = rclpy.create_node("config")
    params = dict()

    param_reader.declare_parameter("algorithm", value="frpn")
    param_reader.declare_parameter("N", value=2.0)
    param_reader.declare_parameter("Vd", value=2.0)
    param_reader.declare_parameter("G", value=2.0)
    param_reader.declare_parameter("W", value=2.0)

    params["algorithm"] = (
        param_reader.get_parameter("algorithm").get_parameter_value().string_value
    )
    params["N"] = param_reader.get_parameter("N").get_parameter_value().double_value
    params["Vd"] = param_reader.get_parameter("Vd").get_parameter_value().double_value
    params["G"] = param_reader.get_parameter("G").get_parameter_value().double_value
    params["W"] = param_reader.get_parameter("W").get_parameter_value().double_value

    param_reader.get_logger().info(
        f"starting the system with following parameters:\n{params}"
    )

    param_reader.destroy_node()
    return params


def strategy_setup(params: dict) -> NavigationStrategy:
    match params["algorithm"]:
        case "ppn":
            return algs.ProportionalNavigation(**params)
        case "tpn":
            return algs.TrueProportionalNavigation(**params)
        case "frpn":
            return algs.FastResponseProportionalNavigation(**params)
        case _:
            return algs.ProportionalNavigation()


def create_behaviour_tree(params) -> Composite:
    startup = Sequence("startup", memory=False)

    establish_connection = behaviours.WaitForConnection("establish_connection")
    get_in_the_air = Selector("get_in_the_air", memory=False)

    is_in_air = behaviours.HeightCheck("is_in_air")
    takeoff = Sequence("takeoff", memory=False)

    ensure_offboard = Selector("ensure_offboard", memory=False)

    is_in_offboard = behaviours.OffboardCheck("is_in_offboard")
    set_offboard_mode = behaviours.OffboardAction("set_offboard_mode")

    ensure_arm = Selector("ensure_arm", memory=False)

    is_armed = behaviours.ArmCheck("is_armed")
    do_arm = behaviours.ArmAction("do_arm")

    gain_altitude = Selector("gain_altitude", memory=False)

    is_height_reached = behaviours.HeightCheck("is_height_reached")
    fly_up = behaviours.TakeoffAction("fly_up")

    system = Selector("system", memory=False)

    # is_mission_finished
    mission = Sequence("mission", memory=False)

    wait_for_start = WaitForData(
        "start_command",
        topic_name="start",
        topic_type=Empty,
        qos_profile=1,
        clearing_policy=ClearingPolicy.NEVER,
    )

    gather_target_data = ToBlackboard(
        "gather_target_data",
        topic_name="/follow_trajectory/_action/feedback",
        topic_type=FollowTrajectory_FeedbackMessage,
        qos_profile=10,
        blackboard_variables={"target": "feedback"},
    )

    intercept_action = behaviours.InterceptAction(
        "intercept", strategy=strategy_setup(params)
    )

    startup.add_child(establish_connection)
    startup.add_child(get_in_the_air)

    get_in_the_air.add_child(is_in_air)
    get_in_the_air.add_child(takeoff)

    takeoff.add_child(ensure_offboard)
    ensure_offboard.add_child(is_in_offboard)
    ensure_offboard.add_child(set_offboard_mode)

    takeoff.add_child(ensure_arm)
    ensure_arm.add_child(is_armed)
    ensure_arm.add_child(do_arm)

    takeoff.add_child(gain_altitude)
    gain_altitude.add_child(is_height_reached)
    gain_altitude.add_child(fly_up)

    startup.add_child(system)

    # system.add_child(is_mission_finished)
    system.add_child(mission)

    mission.add_child(wait_for_start)
    mission.add_child(gather_target_data)
    mission.add_child(intercept_action)

    return startup


def main(args=None):
    rclpy.init()
    params = parameters()
    root = create_behaviour_tree(params)
    tree = BehaviourTree(root=root)
    offboard_control = OffboardControl()
    try:
        tree.setup(timeot=15.0)
    except (TimedOutError, KeyboardInterrupt):
        tree.shutdown()
        rclpy.try_shutdown()

    executor = MultiThreadedExecutor()
    executor.add_node(offboard_control)
    executor.add_node(tree.node)

    tree.tick_tock(period_ms=10.0)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        offboard_control.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
