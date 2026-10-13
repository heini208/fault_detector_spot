"""Behavior-tree command for orienting the tip to its measured target."""

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class OrientSensorTipToSurfaceCommand(ExecutionCommand):
    """Carry attachment identity for current-pose surface discovery/alignment."""

    def __init__(
        self,
        command_id,
        stamp,
        motion_sensor_id: str,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Sensor tip orientation requires active sensor geometry"
            )
        self.motion_sensor_id = sensor_id


__all__ = ["OrientSensorTipToSurfaceCommand"]
