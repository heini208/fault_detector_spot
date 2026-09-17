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
    if intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE:
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
    if intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH:
        pose = point.safe_approach_pose_object
    elif intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH:
        state_source.validate_aligned_probe_distance(
            attachment.motion_sensor_id, point.aligned_preapproach_distance_m,
        )
        pose = point.aligned_preapproach_pose_object
    else:
        raise ValueError("Unsupported saved probe-point motion")
    tag = state_source.reference_tag(definition.reference_tag.tag_id)
    return replace(
        factory.absolute(pose, tag, attachment.motion_sensor_id),
        inspection=selection,
    )
