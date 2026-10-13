"""Behavior-tree command for measuring distance at the current probe pose."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class MeasureSensorTipDistanceCommand(ExecutionCommand):
    """Carry the nominal target range used to find the wall observation patch."""

    def __init__(self, command_id, stamp, expected_distance_m=0.25, request_id=""):
        super().__init__(command_id, stamp, request_id=request_id)
        expected = float(expected_distance_m)
        if not math.isfinite(expected) or expected <= 0.0:
            raise ValueError("Expected sensor tip distance must be positive")
        self.expected_distance_m = expected


__all__ = ["MeasureSensorTipDistanceCommand"]
