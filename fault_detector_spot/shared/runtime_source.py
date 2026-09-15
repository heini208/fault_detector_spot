"""Common lifecycle contract for long-lived runtime state sources."""

from abc import ABC, abstractmethod


class RuntimeSource(ABC):
    """Own long-lived ROS state subscriptions used during operation."""

    @abstractmethod
    def destroy(self) -> None:
        """Destroy ROS resources owned by this source."""


__all__ = ["RuntimeSource"]
