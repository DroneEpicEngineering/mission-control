import rclpy
from rclpy.executors import ExternalShutdownException

from py_trees.composites import Selector, Sequence, Composite

from py_trees_ros.trees import BehaviourTree
from py_trees_ros.exceptions import TimedOutError

from mission_control import behaviours


def create_behaviour_tree() -> Composite:
    startup = Sequence("startup", memory=False)

    ensure_offboard = Selector("ensure_offboard", memory=False)

    is_in_offboard = behaviours.OffboardCheck("is_in_offboard")
    set_offboard_mode = behaviours.OffboardAction("set_offboard_mode")

    ensure_arm = Selector("ensure_arm", memory=False)

    is_armed = behaviours.ArmCheck("is_armed")
    do_arm = behaviours.ArmAction("do_arm")

    takeoff = Selector("takeoff", memory=False)

    is_height_reached = behaviours.HeightCheck("is_height_reached")
    do_takeoff = behaviours.TakeoffAction("do_takeoff")

    startup.add_child(ensure_offboard)
    ensure_offboard.add_child(is_in_offboard)
    ensure_offboard.add_child(set_offboard_mode)

    startup.add_child(ensure_arm)
    ensure_arm.add_child(is_armed)
    ensure_arm.add_child(do_arm)

    startup.add_child(takeoff)
    takeoff.add_child(is_height_reached)
    takeoff.add_child(do_takeoff)

    return startup


def main(args=None):
    rclpy.init(args)
    root = create_behaviour_tree()
    tree = BehaviourTree(root=root)
    try:
        tree.setup(timeot=15.0)
    except TimedOutError | KeyboardInterrupt:
        tree.shutdown()
        rclpy.try_shutdown()

    tree.tick_tock(period_ms=10.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt | ExternalShutdownException:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
