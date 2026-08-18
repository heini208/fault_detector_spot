"""Behavior-tree command for one force-guarded close surface approach."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class MoveCloseToSurfaceCommand(ExecutionCommand):
    """Carry the requested probe-tip stand-off and aligned start distance."""

    def __init__(
        self,
        command_id,
        stamp,
        target_surface_distance_m: float,
        aligned_preapproach_distance_m: float,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        target = float(target_surface_distance_m)
        aligned = float(aligned_preapproach_distance_m)
        if not math.isfinite(target) or target <= 0.0:
            raise ValueError("Target surface distance must be positive")
        if not math.isfinite(aligned) or aligned <= 0.0:
            raise ValueError("Aligned pre-approach distance must be positive")
        if aligned <= target:
            raise ValueError(
                "Aligned pre-approach distance must exceed target surface "
                "distance"
            )
        self.target_surface_distance_m = target
        self.aligned_preapproach_distance_m = aligned


__all__ = ["MoveCloseToSurfaceCommand"]
