from navigation.types import NavigationInput, NavigationOutput
from navigation.navigation_strategy import NavigationStrategy


class NavigationContex:
    def __init__(self, strategy: NavigationStrategy) -> None:
        self._strategy = strategy

    @property
    def strategy(self) -> NavigationStrategy:
        return self._strategy

    @property.setter
    def strategy(self, strategy: NavigationStrategy) -> None:
        self._strategy = strategy

    def execute(self, data: NavigationInput) -> NavigationOutput:
        result = self._strategy.execute(data)
        return result
