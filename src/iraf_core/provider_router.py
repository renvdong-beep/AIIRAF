"""平台无关的 Provider 选择器；不执行控制动作。"""
from dataclasses import dataclass


class ProviderRouteError(ValueError):
    pass


@dataclass(frozen=True)
class ProviderCandidate:
    name: str
    provider_type: str
    capabilities: frozenset = frozenset()
    modes: frozenset = frozenset()
    priority: int = 100


class ProviderRouter:
    def __init__(self, candidates=()):
        items = tuple(candidates)
        for item in items:
            if not item.name or not item.provider_type:
                raise ProviderRouteError("Provider name and type are required")
            if item.priority < 0:
                raise ProviderRouteError("Provider priority must be non-negative")
        self._candidates = items

    def select(self, required_capability, mode=None, provider_type=None):
        if not required_capability:
            raise ProviderRouteError("required capability is required")
        matches = [item for item in self._candidates
                   if required_capability in item.capabilities
                   and (mode is None or not item.modes or mode in item.modes)
                   and (provider_type is None or item.provider_type == provider_type)]
        if not matches:
            raise ProviderRouteError("no Provider matches capability and mode")
        return sorted(matches, key=lambda item: (item.priority, item.name))[0]