from abc import ABC, abstractmethod

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
)


class NavigationStrategy(ABC):
    @property
    def is_ready(self) -> bool:
        return self._state is not None

    @abstractmethod
    def setup(self, data: NavigationInput) -> None:
        pass

    @abstractmethod
    def execute(self, data: NavigationInput) -> NavigationOutput:
        pass
