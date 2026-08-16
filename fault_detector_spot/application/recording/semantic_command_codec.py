from dataclasses import asdict
from typing import List

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    InspectionSelection,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)


def serialize_recorded_command(command: SemanticCommand) -> dict:
    if not isinstance(command, SemanticCommand):
        raise TypeError("Recorded command must be a SemanticCommand")
    data = asdict(command)
    data["command_id"] = command.command_id.value
    data.pop("motion_sensor_id", None)
    return data


def deserialize_recorded_command(data: dict) -> SemanticCommand:
    if not isinstance(data, dict):
        raise TypeError("Recorded command data must be an object")
    try:
        tag_data = data["tag"]
        return SemanticCommand(
            command_id=CommandID(data["command_id"]),
            tag=None if tag_data is None else _tag_from_dict(tag_data),
            offset=_pose_from_dict(data["offset"]),
            orientation_mode=data["orientation_mode"],
            wait_time=data["wait_time"],
            map_name=data["map_name"],
            waypoint_name=data["waypoint_name"],
            inspection=InspectionSelection(
                **_object(data["inspection"], "Inspection selection")
            ),
        )
    except KeyError as exception:
        raise ValueError(
            f"Recorded command is missing field: {exception.args[0]}"
        ) from exception


def deserialize_recording(document) -> List[SemanticCommand]:
    if not isinstance(document, dict):
        raise ValueError("Recording must be an object")
    entries = document.get("commands")
    if not isinstance(entries, list):
        raise ValueError("Recording commands must be a list")
    return [deserialize_recorded_command(entry) for entry in entries]


def _tag_from_dict(data: dict) -> SemanticTag:
    data = _object(data, "Tag")
    return SemanticTag(
        id=data["id"],
        pose=_pose_from_dict(data["pose"]),
    )


def _pose_from_dict(data: dict) -> StampedPose:
    data = _object(data, "Pose")
    return StampedPose(
        frame_id=data["frame_id"],
        stamp_sec=data["stamp_sec"],
        stamp_nanosec=data["stamp_nanosec"],
        position=CommandVector3(
            **_object(data["position"], "Pose position")
        ),
        orientation=CommandQuaternion(
            **_object(data["orientation"], "Pose orientation")
        ),
    )


def _object(value, label):
    if not isinstance(value, dict):
        raise TypeError(f"{label} must be an object")
    return value
