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
    custom_final = intent.intent == OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH
    if custom_final and not point.fully_custom:
        raise ValueError("Custom probe paths require a fully custom probe point")
    if intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE:
        if point.fully_custom:
            raise ValueError("Surface approach requires a surface-relative probe point")
        target = (intent.target_surface_distance_m
                  if intent.override_target_surface_distance
                  else point.target_surface_distance_m)
        validate_surface_distance_pair(target, point.aligned_preapproach_distance_m)
        return SemanticCommand(
            command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
            ignore_environment_collisions=True,
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
        ignore_environment_collisions=custom_final,
        command_id=(CommandID.FOLLOW_MOVE_TO_TAG_PATH
                    if custom_final or intent.intent == OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH
                    else CommandID.MOVE_ARM_TO_TAG),
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
        ignore_environment_collisions=False,
        inspection=InspectionSelection(
            object_id=intent.object_id, routine_id=intent.routine_id,
        ),
    )


def probe_point_plan(command, repository, state_source, attachments, factory):
    """Snapshot saved forward paths; execution backtracks reached checkpoints."""
    selection = command.inspection
    definition = repository.load(selection.object_id)
    definition.validate()
    routine = definition.get_routine(selection.routine_id)
    point = routine.get_probe_point(selection.probe_point_id) if routine else None
    if point is None:
        raise ValueError("The selected probe point does not exist")
    attachment = attachments.require_motion_attachment()
    if not attachment.has_sensor or attachment.motion_sensor_id != command.motion_sensor_id:
        raise ValueError("Probe execution requires the bound physical sensor")

    # Resolve all stages against one immutable definition and tag observation.
    from types import SimpleNamespace
    tag = state_source.reference_tag(routine.reference_tag.tag_id)
    frozen_state = SimpleNamespace(
        reference_tag=lambda _id: tag,
        validate_aligned_probe_distance=state_source.validate_aligned_probe_distance,
    )
    frozen_repository = SimpleNamespace(load=lambda _id: definition)
    frozen_attachments = SimpleNamespace(require_motion_attachment=lambda: attachment)

    def resolve(operation):
        intent = OperationalIntent()
        intent.intent = operation
        intent.object_id = selection.object_id
        intent.routine_id = selection.routine_id
        intent.probe_point_id = selection.probe_point_id
        # Each stage owns its policy; a final probe bypass must not cover travel.
        return saved_probe_command(
            intent, frozen_repository, frozen_state, frozen_attachments, factory,
        )

    safe = resolve(OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH)
    aligned = resolve(OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH)
    measurement = resolve(
        OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH if point.fully_custom
        else OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    )

    return safe, aligned, measurement
