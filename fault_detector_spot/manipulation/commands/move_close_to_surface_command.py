"""Behavior-tree command for a guarded close-surface workflow."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import ManipulatorToTagCommand


class MoveCloseToSurfaceCommand(ExecutionCommand):
    """Carry stand-off target and optional per-command tolerance."""

    def __init__(
        self,
        command_id,
        stamp,
        target_surface_distance_m: float,
        surface_tolerance_m: float = 0.0,
        request_id: str = "",
        expected_distance_m: float | None = None,
        return_target: ManipulatorToTagCommand | None = None,
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
        expected = None if expected_distance_m is None else float(expected_distance_m)
        if expected is not None and (not math.isfinite(expected) or expected <= 0.0):
            raise ValueError("Expected sensor tip distance must be positive")
        self.expected_distance_m = expected
        self.target_surface_distance_m = target
        self.surface_tolerance_m = tolerance
        if return_target is not None and not isinstance(return_target, ManipulatorToTagCommand):
            raise TypeError("Observation return target must be a tag-relative probe command")
        self.return_target = return_target


__all__ = ["MoveCloseToSurfaceCommand"]
