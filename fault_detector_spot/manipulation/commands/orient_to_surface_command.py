"""Behavior-tree command for orienting the active probe to a surface."""

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class OrientToSurfaceCommand(ExecutionCommand):
    """Carry the active motion sensor used for surface orientation."""

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
                "Orient-to-surface command requires active sensor geometry"
            )
        self.motion_sensor_id = sensor_id


__all__ = ["OrientToSurfaceCommand"]
