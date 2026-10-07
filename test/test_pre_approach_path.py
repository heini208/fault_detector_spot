"""Ordered, persistent path authoring through the normal command lane."""

from copy import deepcopy

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState, CommandControllerStatus,
)
from fault_detector_spot.inspection.model.models import ProbePoint
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementStage, RefinementMotionState,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind, ProbeMotionRequest
from test_probe_setup_coordinator import (
    coordinator, create_selected_routine, ImagePoint, pose, approve_all,
)


def begin_setup(probe, commands, approved=False):
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    state = probe.select_reference_pixel(
        state.context, "slot1_hand", ImagePoint(u=20, v=30), "surface_fit", .1, .2,
    )
    if approved:
        return approve_all(probe, commands, state)
    state = probe.begin_refinement(state.context)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_SAFE_APPROACH))
    probe.submit_motion(operation)
    commands.succeed(operation.request)
    return snapshot(probe, state)


def snapshot(probe, state):
    return probe.snapshot(probe.context(state.context.context_id, "probe-ui"))


def add_points(probe, state):
    for name, x in (("Clear housing", .9), ("Above bearing", .75)):
        probe.motion_state_source.pose = pose(x)
        state = probe.add_pathing_point(state.context, name)
    return state


def test_capture_after_final_approval_reorder_and_persist(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands, approved=True)
    final = deepcopy(state.refinement.approved_pose(RefinementStage.ALIGNMENT))
    state = add_points(probe, state)
    state = probe.reorder_pathing_point(state.context, 1, -1)
    assert state.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.NOT_TESTED
    assert state.refinement.approved_pose(RefinementStage.ALIGNMENT) == final
    assert state.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    probe.save_probe_point(state.context, "front", "Front", .01, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("front")
    assert [(p.name, p.pose_object.position.x) for p in point.pre_approach_path] == [
        ("Above bearing", .75), ("Clear housing", .9),
    ]
    assert ProbePoint.from_dict(point.to_dict()).pre_approach_path == point.pre_approach_path
    legacy = point.to_dict()
    del legacy["pre_approach_path"]
    assert ProbePoint.from_dict(legacy).pre_approach_path == []


def test_full_path_visits_each_point_then_final_and_reports_one_result(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands))
    final = state.refinement.candidate_pose(RefinementStage.ALIGNMENT)
    targets = []
    factory = probe.refinement_controller.motion_command_factory
    original = factory.absolute
    def capture(target, *args, **kwargs):
        targets.append(deepcopy(target))
        return original(target, *args, **kwargs)
    factory.absolute = capture
    statuses = []
    probe.add_motion_status_listener(statuses.append)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        ProbeMotionKind.MOVE_PRE_APPROACH_PATH, position_tolerance_m=.025,
    ))
    assert operation.request.command.tag_position_tolerance_m == .025
    probe.submit_motion(operation)
    assert [target.position.x for target in targets] == [final.position.x, .9, .75]
    assert len(operation.request.command.pre_approach_offsets) == 2
    assert operation.request.command.command_id is CommandID.FOLLOW_MOVE_TO_TAG_PATH
    assert operation.request.command.ignore_environment_collisions is False
    with pytest.raises(RuntimeError, match="active motion"):
        probe.add_pathing_point(state.context, "Blocked")
    probe.motion_state_source.pose = deepcopy(final)
    commands.succeed(operation.request)
    assert len(statuses) == 1
    assert statuses[0].request_id == operation.request_id
    assert statuses[0].state is CommandControllerState.SUCCEEDED
    result = snapshot(probe, state)
    assert result.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED
    assert result.refinement.candidate_pose(RefinementStage.ALIGNMENT) == final


@pytest.mark.parametrize("outcome", [CommandControllerState.CANCELLED, CommandControllerState.FAILED])
def test_path_failure_or_cancellation_does_not_reach_final(tmp_path, outcome):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands))
    statuses = []
    probe.add_motion_status_listener(statuses.append)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_PRE_APPROACH_PATH))
    probe.submit_motion(operation)
    count = len(commands.submitted)
    if outcome is CommandControllerState.CANCELLED:
        probe.cancel_motion(state.context, operation.request_id)
    else:
        status = CommandControllerStatus(request=commands.submitted[-1], state=outcome, detail="Motion failed")
        for listener in commands.listeners:
            listener(status)
    assert len(commands.submitted) == count
    assert statuses[-1].request_id == operation.request_id
    assert statuses[-1].state is outcome
    assert snapshot(probe, state).refinement.pending_motion is None


def test_visit_path_point_preserves_final_but_requires_return_to_final(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands, approved=True))
    final = deepcopy(state.refinement.candidate_pose(RefinementStage.ALIGNMENT))
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_PATHING_POINT, pathing_point_index=1))
    probe.submit_motion(operation)
    commands.succeed(operation.request)
    state = snapshot(probe, state)
    assert state.refinement.candidate_pose(RefinementStage.ALIGNMENT) == final
    assert state.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    assert state.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.NOT_TESTED


def test_empty_path_still_moves_to_final(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands)
    before = len(commands.submitted)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_PRE_APPROACH_PATH))
    probe.submit_motion(operation)
    probe.motion_state_source.pose = state.refinement.candidate_pose(RefinementStage.ALIGNMENT)
    commands.succeed(operation.request)
    assert len(commands.submitted) == before + 1
    assert snapshot(probe, state).refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED


def test_snapshot_and_motion_transport_carry_path_data(tmp_path):
    from dataclasses import replace
    from fault_detector_msgs.msg import ProbeSetupState, ProbeSetupMotionIntent
    from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
    from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
    from test_command_request_correlation import FakeClock

    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands))
    message = ProbeSetupStateAdapter(FakeClock()).message(
        replace(state, geometry=None, setup=None), 0, ProbeSetupState.STATE_READY, "Ready",
    )
    assert message.pathing_point_names == ["Clear housing", "Above bearing"]
    assert [p.position.x for p in message.pathing_point_poses_object] == [.9, .75]
    intent = ProbeSetupMotionIntent()
    intent.operation = intent.OPERATION_MOVE_PATHING_POINT
    intent.frame = intent.FRAME_SENSOR
    intent.pathing_point_index = 1
    intent.position_tolerance_m = .01
    intent.orientation_tolerance_rad = .1
    request = ProbeSetupMotionApi._motion_request(intent)
    assert request.kind is ProbeMotionKind.MOVE_PATHING_POINT
    assert request.pathing_point_index == 1
    assert ProbeSetupMotionApi._motion_operation(request.kind) == intent.operation


def test_execution_snapshot_freezes_path_and_resolves_hand_geometry(tmp_path):
    from fault_detector_spot.inspection.execution.probe_execution_session import ProbeExecutionConfiguration
    from test_probe_execution_target import sensor

    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands, approved=True))
    probe.save_probe_point(state.context, "front", "Front", .01, .1, 1.)
    config = ProbeExecutionConfiguration.load("motor", "magnetic_scan", "front", probe.object_repository, sensor())
    target = config.resolve_target(pose(1.))
    assert [p.position.x for p in target.pre_approach_path_probe_poses_execution] == pytest.approx([1.9, 1.75])
    assert [p.position.x for p in target.pre_approach_path_hand_poses_execution] == pytest.approx([1.7, 1.55])


@pytest.mark.parametrize("name", ["Clear housing", " Clear housing ", "CLEAR HOUSING"])
def test_duplicate_path_names_are_rejected_without_changing_draft(tmp_path, name):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands))
    with pytest.raises(ValueError, match="already exists"):
        probe.add_pathing_point(state.context, name)
    current = snapshot(probe, state)
    assert current.context == state.context
    assert current.pre_approach_path == state.pre_approach_path


def test_delete_preserves_final_pose_and_allows_name_reuse(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands, approved=True))
    final = state.refinement.approved_pose(RefinementStage.ALIGNMENT)
    state = probe.delete_pathing_point(state.context, 0)
    assert [point.name for point in state.pre_approach_path] == ["Above bearing"]
    assert state.refinement.approved_pose(RefinementStage.ALIGNMENT) == final
    state = probe.add_pathing_point(state.context, "Clear housing")
    state = probe.delete_pathing_point(state.context, 0)
    state = probe.delete_pathing_point(state.context, 0)
    assert not state.pre_approach_path
    probe.save_probe_point(state.context, "front", "Front", .01, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("front")
    assert point.pre_approach_path == []


def test_delete_rejects_missing_selection_and_active_motion(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = add_points(probe, begin_setup(probe, commands))
    for index in (-1, 2):
        with pytest.raises(ValueError, match="existing pathing point"):
            probe.delete_pathing_point(state.context, index)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(ProbeMotionKind.MOVE_PRE_APPROACH_PATH))
    probe.submit_motion(operation)
    with pytest.raises(RuntimeError, match="active motion"):
        probe.delete_pathing_point(state.context, 0)
    commands.cancel(operation.request_id)
    assert snapshot(probe, state).pre_approach_path == state.pre_approach_path


def test_waypoint_tolerances_persist_and_follow_reordered_points(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands, approved=True)
    for name, tolerance in (("First", .02), ("Second", .04)):
        state = probe.add_pathing_point(state.context, name, tolerance)
    state = probe.reorder_pathing_point(state.context, 1, -1)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        ProbeMotionKind.MOVE_PATHING_POINT, pathing_point_index=0,
        position_tolerance_m=.099,
    ))
    assert operation.request.command.tag_position_tolerance_m == .04
    probe.submit_motion(operation)
    commands.succeed(operation.request)
    state = snapshot(probe, state)
    probe.save_probe_point(state.context, "front", "Front", .03, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("front")
    assert [p.position_tolerance_m for p in point.pre_approach_path] == [.04, .02]
    assert point.position_tolerance_m == .03


def test_saved_final_uses_tolerance_from_alignment_approval(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands, approved=True)
    state = probe.approve_aligned_pose(state.context, .035)
    # Reapprove derived probe geometry after alignment approval.
    state = probe.approve_probe_pose(state.context)
    probe.save_probe_point(state.context, "front", "Front", .099, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("front")
    assert point.position_tolerance_m == .035


def test_path_point_tolerance_defaults_for_existing_saved_data():
    from fault_detector_spot.inspection.model.models import PreApproachPathPoint
    point = PreApproachPathPoint("Existing", pose(.5), .035)
    data = point.to_dict()
    assert PreApproachPathPoint.from_dict(data).position_tolerance_m == .035
    del data["position_tolerance_m"]
    assert PreApproachPathPoint.from_dict(data).position_tolerance_m == .01
