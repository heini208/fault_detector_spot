"""Behavior-tree command for one force-guarded close surface approach."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class MoveCloseToSurfaceCommand(ExecutionCommand):
    """Carry only the requested probe-tip stand-off distance."""

    def __init__(
        self,
        command_id,
        stamp,
        target_surface_distance_m: float,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        distance = float(target_surface_distance_m)
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError("Target surface distance must be positive")
        self.target_surface_distance_m = distance


__all__ = ["MoveCloseToSurfaceCommand"]
