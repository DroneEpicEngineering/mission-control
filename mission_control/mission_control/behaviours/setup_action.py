from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status


class SetupAction(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = self.attach_blackboard_client("setup")
        self._blackboard.register_key("finish", access=Access.WRITE)

    def initialise(self) -> None:
        self._blackboard.set("finish", value=False)

    def update(self) -> Status:
        return Status.SUCCESS
