"""Resolve individual saved-point motions using authoritative object data."""

from dataclasses import replace

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
)

from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    validate_surface_distance_pair,
)


def saved_probe_command(intent, repository, state_source, attachments, factory):
    definition = repository.load(intent.object_id)
    definition.validate()
    routine = definition.get_routine(intent.routine_id)
    if routine is None:
        raise ValueError("The selected routine does not exist for this object")
    point = routine.get_probe_point(intent.probe_point_id)
    if point is None:
        raise ValueError("The selected probe point does not exist in this routine")
    point.validate()
    if attachments is None:
        raise RuntimeError("Sensor attachment state is unavailable")
    attachment = attachments.require_motion_attachment()
    selection = InspectionSelection(
        object_id=intent.object_id,
        routine_id=intent.routine_id,
        probe_point_id=intent.probe_point_id,
    )
    custom_final = point.fully_custom and intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    if intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE and not point.fully_custom:
        target = (intent.target_surface_distance_m
                  if intent.override_target_surface_distance
                  else point.target_surface_distance_m)
        validate_surface_distance_pair(target, point.aligned_preapproach_distance_m)
        return SemanticCommand(
            command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
            target_surface_distance_m=target,
            aligned_preapproach_distance_m=point.aligned_preapproach_distance_m,
            motion_sensor_id=attachment.motion_sensor_id,
            inspection=selection,
        )
    if state_source is None:
        raise RuntimeError("Live robot pose data is unavailable")
    if custom_final:
        pose = point.final_probe_pose_object
        tolerance = point.final_position_tolerance_m
    elif intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH:
        pose = routine.require_safe_approach_pose()
        tolerance = routine.safe_approach_position_tolerance_m
    elif intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH:
        if not point.fully_custom:
            state_source.validate_aligned_probe_distance(
                attachment.motion_sensor_id, point.aligned_preapproach_distance_m,
            )
        pose = point.aligned_preapproach_pose_object
        tolerance = point.position_tolerance_m
    else:
        raise ValueError("Unsupported saved probe-point motion")
    tag = state_source.reference_tag(routine.reference_tag.tag_id)
    offsets = ()
    path = point.final_probe_path if custom_final else point.pre_approach_path
    if custom_final or intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH:
        offsets = tuple(
            factory.absolute(waypoint.pose_object, tag, attachment.motion_sensor_id).offset
            for waypoint in path
        )
    return replace(
        factory.absolute(pose, tag, attachment.motion_sensor_id),
        pre_approach_offsets=offsets,
        pre_approach_speed_scales=tuple(p.arm_speed_scale for p in path) if offsets else (),
        arm_speed_scale=(point.final_probe_speed_scale if custom_final else
                         point.pre_approach_speed_scale if intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH else 1.0),
        pre_approach_tolerances_m=(
            tuple(p.position_tolerance_m for p in path)
            if offsets else ()
        ),
        tag_position_tolerance_m=tolerance,
        inspection=selection,
    )


def routine_safe_approach_command(
    intent, repository, state_source, attachments, factory,
):
    """Resolve the shared routine pose without requiring a probe point."""
    if intent.intent != OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH:
        raise ValueError("Unsupported routine safe-approach motion")
    definition = repository.load(intent.object_id)
    definition.validate()
    routine = definition.get_routine(intent.routine_id)
    if routine is None:
        raise ValueError("The selected routine does not exist for this object")
    pose = routine.require_safe_approach_pose()
    if attachments is None:
        raise RuntimeError("Sensor attachment state is unavailable")
    attachment = attachments.require_motion_attachment()
    if state_source is None:
        raise RuntimeError("Live robot pose data is unavailable")
    tag = state_source.reference_tag(routine.reference_tag.tag_id)
    if int(tag.id) != routine.reference_tag.tag_id:
        raise ValueError("Live reference tag does not match the selected routine")
    return replace(
        factory.absolute(
            pose, tag, attachment.motion_sensor_id, safe_approach=True,
        ),
        tag_position_tolerance_m=routine.safe_approach_position_tolerance_m,
        inspection=InspectionSelection(
            object_id=intent.object_id, routine_id=intent.routine_id,
        ),
    )
