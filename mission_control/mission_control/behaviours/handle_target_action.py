from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from system_interfaces.action import FollowTrajectory
from system_interfaces.action._follow_trajectory import FollowTrajectory_FeedbackMessage


class HandleTargetAction(Behaviour):
    def __init__(self, name: str, node: Node) -> None:
        super().__init__(name)
        self._node = node
        self._action_client: ActionClient = None
        self._send_goal_future: Future = None
        self._target_data: FollowTrajectory.Feedback = None
        self._last_target_data: FollowTrajectory.Feedback = None
        self._action_result: FollowTrajectory.Result = None

        self._blackboard = self.attach_blackboard_client("handle_target")
        self._blackboard.register_key("target", access=Access.WRITE)
        self._blackboard.register_key("finish", access=Access.WRITE)

    def setup(self, **kwargs) -> None:
        self._action_client = ActionClient(
            self._node, FollowTrajectory, "follow_trajectory"
        )

    def initialise(self) -> None:
        goal_msg = FollowTrajectory.Goal()
        goal_msg.trajectory_index = 1

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.__update_target_data
        )
        self._send_goal_future.add_done_callback(self.__handle_response)

    def update(self) -> Status:
        if (
            self._last_target_data is None
            or self._last_target_data.header.stamp.sec
            == self._target_data.header.stamp.sec
        ):
            return Status.RUNNING

        self._blackboard.set("target", self._target_data)
        self._last_target_data = self._target_data

        return Status.SUCCESS

    def __update_target_data(self, msg: FollowTrajectory_FeedbackMessage) -> None:
        if self._last_target_data is None:
            self._last_target_data = msg.feedback

        self._target_data = msg.feedback

    def __handle_response(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__handle_result)

    def __handle_result(self, future: Future) -> None:
        self._action_result = future.result().result
        self._blackboard.set("finish", value=True)
