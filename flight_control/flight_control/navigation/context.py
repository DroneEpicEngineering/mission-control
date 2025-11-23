from flight_control.navigation.types import NavigationInput, NavigationOutput
from flight_control.navigation.strategy import NavigationStrategy


class NavigationContext:
    def __init__(self, strategy: NavigationStrategy) -> None:
        self._strategy = strategy

    @property
    def strategy(self) -> NavigationStrategy:
        return self._strategy

    @strategy.setter
    def strategy(self, strategy: NavigationStrategy) -> None:
        self._strategy = strategy

    def execute(self, data: NavigationInput) -> NavigationOutput:
        if not self._strategy.is_ready:
            self._strategy.setup(data)

        return self._strategy.execute(data)
