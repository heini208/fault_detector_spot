"""Behavior-tree command for a guarded close-surface workflow."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class MoveCloseToSurfaceCommand(ExecutionCommand):
    """Carry stand-off target and optional per-command tolerance."""

    def __init__(
        self,
        command_id,
        stamp,
        target_surface_distance_m: float,
        surface_tolerance_m: float = 0.0,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        target = float(target_surface_distance_m)
        tolerance = float(surface_tolerance_m)
        if not math.isfinite(target) or target < 0.0:
            raise ValueError(
                "Target surface distance must be non-negative"
            )
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError(
                "Surface tolerance must be non-negative"
            )
        self.target_surface_distance_m = target
        self.surface_tolerance_m = tolerance


__all__ = ["MoveCloseToSurfaceCommand"]
