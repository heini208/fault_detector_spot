"""Per-tag cache for timestamped tag observations."""

from copy import deepcopy
from typing import Dict

from fault_detector_msgs.msg import TagElement
from rclpy.time import Time

from .tag_observation_time import (
    is_observation_fresh,
)


class TagObservationCache:
    """Keep the newest observation for each tag until it expires."""

    def __init__(self, max_age_sec: float):
        """Create a cache with the given observation lifetime."""
        if max_age_sec < 0.0:
            raise ValueError("max_age_sec must be non-negative")

        self.max_age_sec = max_age_sec
        self._observations: Dict[int, TagElement] = {}

    def update(
        self,
        observations: Dict[int, TagElement],
    ) -> None:
        """Merge observations without clearing tags omitted by this update."""
        for tag_id, observation in observations.items():
            existing = self._observations.get(tag_id)

            if (
                existing is not None
                and self._stamp_key(observation)
                < self._stamp_key(existing)
            ):
                continue

            self._observations[tag_id] = deepcopy(observation)

    def snapshot(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        """Return fresh observations and remove expired entries."""
        expired_ids = [
            tag_id
            for tag_id, observation in self._observations.items()
            if not is_observation_fresh(
                current_time,
                observation.pose.header.stamp,
                self.max_age_sec,
            )
        ]

        for tag_id in expired_ids:
            del self._observations[tag_id]

        return deepcopy(self._observations)

    def clear(self) -> None:
        """Remove every cached observation."""
        self._observations.clear()

    @staticmethod
    def _stamp_key(observation: TagElement):
        stamp = observation.pose.header.stamp
        return stamp.sec, stamp.nanosec
