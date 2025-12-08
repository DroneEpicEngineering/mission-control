from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access


class FinishedCheck(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = self.attach_blackboard_client("finish_check")
        self._blackboard.register_key("finish", access=Access.READ)

    def update(self) -> Status:
        return Status.SUCCESS if self._blackboard.get("finish") else Status.FAILURE
