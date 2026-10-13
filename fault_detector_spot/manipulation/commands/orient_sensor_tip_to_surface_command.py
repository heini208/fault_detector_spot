"""Behavior-tree command for orienting the tip to its measured target."""

import math

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class OrientSensorTipToSurfaceCommand(ExecutionCommand):
    """Carry attachment identity and the nominal target observation range."""

    def __init__(
        self,
        command_id,
        stamp,
        motion_sensor_id: str,
        expected_distance_m: float = 0.25,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Sensor tip orientation requires active sensor geometry"
            )
        expected = float(expected_distance_m)
        if not math.isfinite(expected) or expected <= 0.0:
            raise ValueError("Expected sensor tip distance must be positive")
        self.motion_sensor_id = sensor_id
        self.expected_distance_m = expected


__all__ = ["OrientSensorTipToSurfaceCommand"]
