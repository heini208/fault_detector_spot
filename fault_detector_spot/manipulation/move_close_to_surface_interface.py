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
    aligned_preapproach_distance_m: float

    def __post_init__(self):
        request_id = self.request_id.strip()
        target = float(self.target_surface_distance_m)
        aligned = float(self.aligned_preapproach_distance_m)
        if not request_id:
            raise ValueError("Close-surface request ID must not be empty")
        if not math.isfinite(target) or target <= 0.0:
            raise ValueError("Target surface distance must be positive")
        if not math.isfinite(aligned) or aligned <= 0.0:
            raise ValueError("Aligned pre-approach distance must be positive")
        if aligned <= target:
            raise ValueError(
                "Aligned pre-approach distance must exceed target surface "
                "distance"
            )
        object.__setattr__(self, "request_id", request_id)
        object.__setattr__(self, "target_surface_distance_m", target)
        object.__setattr__(self, "aligned_preapproach_distance_m", aligned)


__all__ = [
    "MOVE_CLOSE_TO_SURFACE_ACTION",
    "MoveCloseToSurfaceRequest",
]
