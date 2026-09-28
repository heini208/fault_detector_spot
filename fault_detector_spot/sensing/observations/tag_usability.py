"""Proximity filtering for fresh tag observations in Spot's body frame."""

import math


class TagUsabilityFilter:
    """Apply a rough sensing range, independently of arm reachability."""

    def __init__(self, maximum_range_m: float):
        maximum_range_m = float(maximum_range_m)
        if not math.isfinite(maximum_range_m) or maximum_range_m <= 0.0:
            raise ValueError("Maximum tag range must be positive and finite")
        self.maximum_range_m = maximum_range_m

    def filter(self, observations):
        """Keep observations within range of their common body-frame origin."""
        usable = {}
        for tag_id, tag in observations.items():
            position = tag.pose.pose.position
            distance = math.hypot(
                float(position.x), float(position.y), float(position.z)
            )
            if distance <= self.maximum_range_m:
                usable[int(tag_id)] = tag
        return usable


__all__ = ["TagUsabilityFilter"]
