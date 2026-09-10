"""Pure reachability filtering for current tag observations."""

import math


class TagReachabilityFilter:
    """Filter tag observations by distance from the arm shoulder origin."""

    def __init__(self, maximum_reach_m: float):
        maximum_reach_m = float(maximum_reach_m)
        if not math.isfinite(maximum_reach_m) or maximum_reach_m <= 0.0:
            raise ValueError("Maximum arm reach must be positive and finite")
        self.maximum_reach_m = maximum_reach_m

    def filter(self, observations, arm_base_position):
        if arm_base_position is None:
            return {}

        try:
            x_offset, y_offset, z_offset = (
                float(value) for value in arm_base_position
            )
        except (TypeError, ValueError) as exception:
            raise ValueError(
                "Arm base position must contain three finite values"
            ) from exception

        values = (x_offset, y_offset, z_offset)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Arm base position must contain three finite values"
            )

        reachable = {}
        for tag_id, tag in observations.items():
            position = tag.pose.pose.position
            dx = float(position.x) - x_offset
            dy = float(position.y) - y_offset
            dz = float(position.z) - z_offset
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= self.maximum_reach_m:
                reachable[int(tag_id)] = tag
        return reachable


__all__ = ["TagReachabilityFilter"]
