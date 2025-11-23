from abc import ABC, abstractmethod

from flight_control.navigation.types import (
    NavigationInput,
    NavigationOutput,
)


class NavigationStrategy(ABC):
    def __init__(self, a_max=1.0) -> None:
        self._a_max = a_max

    @property
    def is_ready(self) -> bool:
        return self._state is not None

    @abstractmethod
    def setup(self, data: NavigationInput) -> None:
        pass

    @abstractmethod
    def execute(self, data: NavigationInput) -> NavigationOutput:
        pass
