"""Resolve a saved routine base position into existing base-to-tag motion."""

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    InspectionSelection,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)


def routine_base_position_command(intent, repository, state_source):
    """Build base motion and its final body height from authoritative data."""
    if (
        intent.intent
        != OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION
    ):
        raise ValueError("Unsupported routine base-position motion")

    definition = repository.load(intent.object_id)
    definition.validate()
    routine = definition.get_routine(intent.routine_id)
    if routine is None:
        raise ValueError(
            "The selected routine does not exist for this object"
        )
    if routine.base_position is None:
        raise ValueError(
            "The selected routine has no configured base position"
        )
    routine.base_position.validate()
    if state_source is None:
        raise RuntimeError("Live robot pose data is unavailable")

    tag_id = routine.reference_tag.tag_id
    reference_tag = state_source.reference_tag(tag_id)
    if int(reference_tag.id) != tag_id:
        raise ValueError(
            "Live reference tag does not match the selected routine"
        )
    frame_id = reference_tag.pose.header.frame_id.strip()
    if not frame_id:
        raise ValueError("Reference tag pose frame is empty")

    tag_pose = reference_tag.pose
    base_position = routine.base_position
    return SemanticCommand(
        command_id=CommandID.MOVE_BASE_TO_TAG,
        tag=SemanticTag(
            id=tag_id,
            pose=StampedPose(
                frame_id=frame_id,
                stamp_sec=int(tag_pose.header.stamp.sec),
                stamp_nanosec=int(tag_pose.header.stamp.nanosec),
                position=CommandVector3(
                    x=tag_pose.pose.position.x,
                    y=tag_pose.pose.position.y,
                    z=tag_pose.pose.position.z,
                ),
                orientation=CommandQuaternion(
                    x=tag_pose.pose.orientation.x,
                    y=tag_pose.pose.orientation.y,
                    z=tag_pose.pose.orientation.z,
                    w=tag_pose.pose.orientation.w,
                ),
            ),
        ),
        offset=StampedPose(
            frame_id=f"Tag_{tag_id}",
            position=CommandVector3(
                x=base_position.position.x,
                y=base_position.position.y,
                z=base_position.position.z,
            ),
            orientation=CommandQuaternion(
                x=base_position.orientation.x,
                y=base_position.orientation.y,
                z=base_position.orientation.z,
                w=base_position.orientation.w,
            ),
        ),
        walking_profile=intent.walking_profile,
        body_height_m=routine.base_body_height_m,
        inspection=InspectionSelection(
            object_id=intent.object_id,
            routine_id=intent.routine_id,
        ),
    )


__all__ = ["routine_base_position_command"]
