"""Behavior-tree command for measuring distance at the current probe pose."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import ManipulatorToTagCommand


class MeasureSensorTipDistanceCommand(ExecutionCommand):
    """Measure current distance, optionally checking a saved goal's wall range."""

    def __init__(self, command_id, stamp, expected_distance_m=None, request_id="",
                 return_target: ManipulatorToTagCommand | None = None):
        super().__init__(command_id, stamp, request_id=request_id)
        expected = None if expected_distance_m is None else float(expected_distance_m)
        if expected is not None and (not math.isfinite(expected) or expected <= 0.0):
            raise ValueError("Expected sensor tip distance must be positive")
        self.expected_distance_m = expected
        if return_target is not None and not isinstance(return_target, ManipulatorToTagCommand):
            raise TypeError("Observation return target must be a tag-relative probe command")
        self.return_target = return_target


__all__ = ["MeasureSensorTipDistanceCommand"]
