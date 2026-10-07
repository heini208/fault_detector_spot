"""Custom authoring keeps explicit poses, independent paths and bounded speeds."""
from copy import deepcopy
from dataclasses import replace

import pytest

from fault_detector_msgs.msg import ProbeSetupState
from fault_detector_spot.inspection.model.models import ProbePoint, PreApproachPathPoint
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage, RefinementMotionState
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind, ProbeMotionRequest
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
from fault_detector_spot.ui.ros.probe_setup_state_adapter import probe_setup_state_to_view
from test_command_request_correlation import FakeClock
from test_probe_setup_coordinator import coordinator, create_selected_routine, pose


def begin_custom(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    # Custom mode must never ask the depth/calculated-geometry owner for a target.
    probe.geometry_editor.geometry = None
    state = probe.begin_refinement(state.context, fully_custom=True)
    return probe, commands, state


def latest(probe, state):
    return probe.snapshot(probe.context(state.context.context_id, "probe-ui"))


def move(probe, commands, state, kind, stage="alignment", achieved=None, **kwargs):
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(kind, path_stage=stage, **kwargs))
    probe.submit_motion(operation)
    if achieved is not None:
        probe.motion_state_source.pose = achieved
    commands.succeed(operation.request)
    return latest(probe, state), operation.request.command


def test_custom_starts_without_reference_or_default_candidates(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    assert state.geometry is None and state.reference_pixel is None
    assert state.setup.surface_target is None
    assert state.refinement.candidate_pose(RefinementStage.ALIGNMENT) is None
    assert state.refinement.candidate_pose(RefinementStage.PROBE) is None
    with pytest.raises(RuntimeError, match="preceding"):
        probe.approve_aligned_pose(state.context)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    for kind in (ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH, ProbeMotionKind.MOVE_PRE_APPROACH_PATH):
        with pytest.raises(ValueError, match="Save a candidate"):
            probe.prepare_motion(state.context, ProbeMotionRequest(kind))
    message = ProbeSetupStateAdapter(FakeClock()).message(state, 0, ProbeSetupState.STATE_READY, "Ready")
    assert message.fully_custom and not message.has_alignment_candidate
    view = probe_setup_state_to_view(message)
    assert view.refinement.candidate_pose(RefinementStage.PROBE) is None
    assert view.surface_target is None


def test_custom_capture_paths_save_and_replay_preserve_independent_geometry(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    probe.motion_state_source.pose = pose(.7, .2, .1)
    state = probe.add_pathing_point(state.context, "pre waypoint", .025, .7)
    probe.motion_state_source.pose = pose(.8, .1, .2)
    state = probe.approve_aligned_pose(state.context, .02, .8)
    aligned = deepcopy(state.setup.aligned_preapproach_pose_object)
    probe.motion_state_source.pose = pose(.9, .4, .3)
    state = probe.add_pathing_point(state.context, "final waypoint", .015, .15, "probe")
    probe.motion_state_source.pose = pose(1., .5, .4)
    state = probe.approve_probe_pose(state.context, .005, .2)
    assert state.setup.aligned_preapproach_pose_object == aligned
    state, command = move(probe, commands, state, ProbeMotionKind.MOVE_PRE_APPROACH_PATH,
                          "probe", achieved=state.setup.probe_pose_object)
    assert command.pre_approach_speed_scales == (.15,)
    assert command.arm_speed_scale == .2
    assert command.pre_approach_tolerances_m == (.015,)
    state = probe.begin_finalization(state.context, "9b361339-3d85-4a2e-a5c3-88cf9a6d4872", True)
    state = probe.approve_probe_geometry_for_finalization(state.context, "9b361339-3d85-4a2e-a5c3-88cf9a6d4872")
    state = probe.save_probe_point_for_finalization(state.context, "9b361339-3d85-4a2e-a5c3-88cf9a6d4872", "custom", "Custom", .01, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("custom")
    point.validate()
    restored = ProbePoint.from_dict(point.to_dict())
    assert restored == point
    assert point.final_probe_pose_object == pose(1., .5, .4)
    assert point.aligned_preapproach_pose_object == aligned
    assert [p.name for p in point.pre_approach_path] == ["pre waypoint"]
    assert [p.name for p in point.final_probe_path] == ["final waypoint"]
    assert point.pre_approach_speed_scale == .8
    assert point.final_position_tolerance_m == .005
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH, retract_path=True),
        finalization_request_id="9b361339-3d85-4a2e-a5c3-88cf9a6d4872")
    assert operation.request.command.pre_approach_speed_scales == (.15,)
    assert operation.request.command.ignore_environment_collisions is True


@pytest.mark.parametrize("speed", [0, -1, 1.01, float("nan"), float("inf")])
def test_invalid_speed_is_rejected_before_capturing_pose(tmp_path, speed):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    with pytest.raises(ValueError, match="speed"):
        probe.approve_aligned_pose(state.context, .01, speed)
    assert latest(probe, state).refinement.candidate_pose(RefinementStage.ALIGNMENT) is None
    with pytest.raises(ValueError, match="speed"):
        probe.add_pathing_point(state.context, "bad", .01, speed)


def test_custom_large_adjustment_still_uses_guarded_command_lane(tmp_path):
    from fault_detector_spot.shared.geometry.models import Vector3Data
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    state, command = move(probe, commands, state, ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
                          achieved=pose(.2), translation=Vector3Data(.2, 0, 0), arm_speed_scale=.3)
    assert command.arm_speed_scale == .3
    assert command.ignore_environment_collisions is False
    assert state.refinement.candidate_pose(RefinementStage.ALIGNMENT) == pose(.2)
    assert not state.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    with pytest.raises(ValueError, match="exceeds"):
        probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.ADJUST_PATHING_POSE,
                             translation=Vector3Data(.201, 0, 0)))
    with pytest.raises(ValueError, match="unavailable"):
        probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.ORIENT_TO_SURFACE))


def test_each_path_speed_can_be_changed_without_recapturing_pose(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    probe.motion_state_source.pose = pose(.8)
    state = probe.add_pathing_point(state.context, "pre", .01, .7)
    original = deepcopy(state.pre_approach_path[0].pose_object)
    probe.motion_state_source.pose = pose(.2)
    state = probe.set_pathing_point_speed(state.context, 0, .45)
    assert state.pre_approach_path[0].pose_object == original
    assert state.pre_approach_path[0].arm_speed_scale == .45
    with pytest.raises(ValueError, match="speed"):
        probe.set_pathing_point_speed(state.context, 0, 1.1)
    assert latest(probe, state).pre_approach_path[0].arm_speed_scale == .45


def test_saved_custom_final_command_uses_final_path_without_surface_validation():
    from unittest.mock import Mock
    from fault_detector_msgs.msg import OperationalIntent, TagElement
    from fault_detector_spot.inspection.execution.saved_probe_motion import saved_probe_command
    from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
    from test_probe_execution_target import inspection_object, sensor
    from test_saved_probe_controls import saved_intent
    definition = inspection_object()
    point = definition.routines[0].probe_points[0]
    point.fully_custom = True
    point.target_surface_distance_m = point.aligned_preapproach_distance_m = 0.
    point.final_probe_pose_object = pose(.9, .4, .3)
    point.final_probe_path = [PreApproachPathPoint("final", pose(.7), .02, .15)]
    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.
    source = Mock(reference_tag=Mock(return_value=tag))
    command = saved_probe_command(
        saved_intent(OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH),
        Mock(load=Mock(return_value=definition)), source,
        Mock(require_motion_attachment=Mock(return_value=sensor())), ProbeSetupMotionCommandFactory(),
    )
    assert command.offset.position.x == .9
    assert command.offset.position.y == .4
    assert command.pre_approach_speed_scales == (.15,)
    assert command.arm_speed_scale == .2
    assert command.ignore_environment_collisions is True
    source.validate_aligned_probe_distance.assert_not_called()


@pytest.mark.parametrize("operation", [
    "INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH",
    "INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH",
    "INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH",
])
def test_saved_approach_travel_follows_global_policy_even_with_parent_bypass(operation):
    from unittest.mock import Mock
    from fault_detector_msgs.msg import OperationalIntent, TagElement
    from fault_detector_spot.inspection.execution.saved_probe_motion import (
        routine_safe_approach_command, saved_probe_command,
    )
    from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
    from test_probe_execution_target import inspection_object, sensor
    from test_saved_probe_controls import saved_intent

    intent = saved_intent(getattr(OperationalIntent, operation))
    intent.ignore_environment_collisions = True
    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    resolver = (routine_safe_approach_command if operation == "INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH"
                else saved_probe_command)
    command = resolver(
        intent, Mock(load=Mock(return_value=inspection_object())),
        Mock(reference_tag=Mock(return_value=tag)),
        Mock(require_motion_attachment=Mock(return_value=sensor())),
        ProbeSetupMotionCommandFactory(),
    )
    assert command.ignore_environment_collisions is False


@pytest.mark.parametrize("terminal", ["succeeded", "failed", "cancelled"])
@pytest.mark.parametrize("intent_name", ["INTENT_MOVE_ARM_RELATIVE", "INTENT_STOW_ARM", "INTENT_SIT_DOWN"])
def test_external_arm_motion_invalidates_reached_state_but_preserves_candidate(tmp_path, terminal, intent_name):
    from fault_detector_msgs.msg import OperationalIntent
    from fault_detector_spot.application.controllers.application_controller import ApplicationController
    from fault_detector_spot.application.controllers.command_controller import CommandControllerState, CommandControllerStatus

    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    probe.motion_state_source.pose = pose(.8)
    state = probe.approve_aligned_pose(state.context)
    approved = deepcopy(state.setup)
    updates = []
    probe.add_state_listener(updates.append)
    app = ApplicationController(commands)
    intent = OperationalIntent()
    intent.intent = getattr(OperationalIntent, intent_name)
    intent.offset.header.frame_id = "body"
    intent.offset.pose.position.x = .1
    intent.offset.pose.orientation.w = 1.
    operation = app.prepare_operation(intent, "probe-ui")
    app.submit(operation)
    for listener in tuple(commands.listeners):
        listener(CommandControllerStatus(operation.request, CommandControllerState.DISPATCHED))
    state = latest(probe, state)
    assert state.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.NOT_TESTED
    assert state.refinement.motion_states[RefinementStage.PROBE] is RefinementMotionState.NOT_TESTED
    assert not state.refinement.alignment_candidate_reached
    assert state.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    assert state.setup == approved
    assert len(updates) == 1
    probe.motion_state_source.pose = pose(.9)
    for listener in tuple(commands.listeners):
        listener(CommandControllerStatus(operation.request, CommandControllerState(terminal)))
    assert len(updates) == 1
    # Capture after manual positioning remains possible without revisiting safe.
    state = probe.approve_aligned_pose(state.context)
    assert state.setup.aligned_preapproach_pose_object.position.x == .9
    assert state.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED


def test_unrelated_command_does_not_invalidate_reached_candidate(tmp_path):
    from fault_detector_spot.application.commanding.command_ids import CommandID
    from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
    from fault_detector_spot.application.commanding.command_request import CommandRequest, CommandOrigin, RecordingPolicy
    from fault_detector_spot.application.controllers.command_controller import CommandControllerState, CommandControllerStatus

    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    state = probe.approve_aligned_pose(state.context)
    request = CommandRequest.create(command=SemanticCommand(command_id=CommandID.WAIT_TIME),
                                    client_id="probe-ui", origin=CommandOrigin.OPERATIONAL,
                                    recording_policy=RecordingPolicy.EXCLUDE)
    for listener in tuple(commands.listeners):
        listener(CommandControllerStatus(request, CommandControllerState.DISPATCHED))
    assert latest(probe, state).refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED
