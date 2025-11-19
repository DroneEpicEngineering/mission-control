from abc import ABC, abstractmethod
from navigation.types import NavigationState, NavigationInput, NavigationOutput


class NagivationStrategy(ABC):
    def __init__(self) -> None:
        self._state = NavigationState()
        pass

    @abstractmethod
    def execute(self, data: NavigationInput) -> NavigationOutput:
        pass
