"""Shared names for the close-surface ROS action boundary."""

from dataclasses import dataclass
import math


MOVE_CLOSE_TO_SURFACE_ACTION = (
    "fault_detector/manipulation/move_close_to_surface"
)


@dataclass(frozen=True)
class MoveCloseToSurfaceRequest:
    """Validated domain request executed by the standalone server."""

    request_id: str
    target_surface_distance_m: float

    def __post_init__(self):
        request_id = self.request_id.strip()
        distance = float(self.target_surface_distance_m)
        if not request_id:
            raise ValueError("Close-surface request ID must not be empty")
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError("Target surface distance must be positive")
        object.__setattr__(self, "request_id", request_id)
        object.__setattr__(self, "target_surface_distance_m", distance)


__all__ = [
    "MOVE_CLOSE_TO_SURFACE_ACTION",
    "MoveCloseToSurfaceRequest",
]
