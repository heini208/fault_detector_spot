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


def routine_base_position_command(intent, repository):
    """Snapshot saved motion settings; resolve the live tag during execution."""
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

    tag_id = routine.reference_tag.tag_id
    base_position = routine.base_position
    return SemanticCommand(
        command_id=CommandID.MOVE_BASE_TO_TAG,
        # BaseMotionPlanner reads the visible tag after walking readiness.
        # An unresolved pose allows admission while earlier commands are moving.
        tag=SemanticTag(
            id=tag_id,
            pose=StampedPose(),
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
