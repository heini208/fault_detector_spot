"""Behavior-tree command for orienting the active probe to a tag."""

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)


class OrientToTagCommand(ExecutionCommand):
    """Carry the selected tag and active motion sensor for tag orientation."""

    def __init__(
        self,
        command_id,
        stamp,
        tag_id: int,
        motion_sensor_id: str,
        request_id: str = "",
    ):
        super().__init__(command_id, stamp, request_id=request_id)
        normalized_tag_id = int(tag_id)
        if normalized_tag_id < 0:
            raise ValueError("Orient-to-tag command requires a valid tag ID")
        sensor_id = str(motion_sensor_id).strip()
        if not sensor_id:
            raise ValueError(
                "Orient-to-tag command requires active sensor geometry"
            )
        self.tag_id = normalized_tag_id
        self.motion_sensor_id = sensor_id


__all__ = ["OrientToTagCommand"]
