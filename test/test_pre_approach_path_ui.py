"""Path popup state and intents, independent of a running robot."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupMotionIntent, ProbeSetupState
from test_probe_point_guided_workflow import application, FakeUI
from test_probe_safe_approach_navigation import safe_state
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage


def test_popup_captures_names_reorders_and_emits_selected_move(application):
    ui = FakeUI()
    motions = []
    ui.probe_setup_client.execute_motion = lambda intent: motions.append(intent) or "motion"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.safe_approach_motion_state = ProbeSetupState.MOTION_REACHED
    state.alignment_motion_state = ProbeSetupState.MOTION_REACHED
    state.surface_alignment_approved = True
    state.pathing_point_names = ["Clear housing", "Above bearing"]
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert dialog.pathing_point_count_label.text() == "Pathing points: 2"
    dialog.add_pathing_point_button.click()
    popup = dialog.path_dialog
    assert popup.isVisible()
    popup.points.setCurrentRow(1)
    assert popup.up_button.isEnabled()
    assert not popup.down_button.isEnabled()
    popup.up_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_REORDER_PATHING_POINT
    assert ui.requests[-1].pathing_point_index == 1
    assert ui.requests[-1].pathing_point_direction == -1
    state.pathing_point_names = ["Above bearing", "Clear housing"]
    controls.apply_setup_state(state)
    assert popup.points.currentRow() == 0
    popup.move_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_PATHING_POINT
    assert motions[-1].pathing_point_index == 0
    controls.apply_setup_state(state)
    popup.name_field.setText("Third point")
    popup.tolerance_field.setValue(.035)
    popup.add_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_ADD_PATHING_POINT
    assert ui.requests[-1].pathing_point_name == "Third point"
    assert ui.requests[-1].position_tolerance_m == .035
    controls.apply_setup_state(state)
    dialog.move_path_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_PRE_APPROACH_PATH
    popup.close_button.click()
    assert not popup.isVisible()
    dialog.hide()


def test_path_popup_disables_edits_during_movement(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = ProbeSetupState.MOTION_REACHED
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    state.pathing_point_names = ["Point"]
    dialog.path_dialog.name_field.setText("Next")
    dialog.refresh_path_controls(True)
    assert dialog.path_dialog.add_button.isEnabled()
    dialog.refresh_path_controls(False)
    assert not dialog.path_dialog.add_button.isEnabled()
    assert not dialog.path_dialog.move_button.isEnabled()
    assert not dialog.move_path_button.isEnabled()
    dialog.hide()


def test_delete_selection_and_duplicate_name_feedback(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.pathing_point_names = ["Clear housing"]
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    popup = dialog.path_dialog
    popup.name_field.setText(" CLEAR HOUSING ")
    assert not popup.add_button.isEnabled()
    assert "already exists" in popup.name_error_label.text()
    popup.points.setCurrentRow(-1)
    assert not popup.delete_button.isEnabled()
    popup.points.setCurrentRow(0)
    assert popup.delete_button.isEnabled()
    popup.delete_button.click()
    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_DELETE_PATHING_POINT
    assert intent.pathing_point_index == 0
    assert not popup.delete_button.isEnabled()
    state.pathing_point_names = []
    controls.apply_setup_state(state)
    assert dialog.pathing_point_count_label.text() == "Pathing points: 0"
    assert popup.add_button.isEnabled()
    assert not popup.delete_button.isEnabled()
    assert not popup.name_error_label.text()
    dialog.hide()


def test_final_tolerance_is_only_shown_in_alignment_and_sent_on_save(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show()
    dialog.workflow_stack.setCurrentIndex(dialog.REFERENCE_PAGE)
    assert not controls.probe_position_tolerance_field.isVisible()
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert controls.probe_position_tolerance_field.isVisible()
    controls.probe_position_tolerance_field.setText("0.035")
    controls.handle_use_current_alignment()
    assert controls.ui.requests[-1].position_tolerance_m == .035
    dialog.hide()


@pytest.mark.parametrize("achieved_pose", ["reached", "off_target", "unavailable"])
def test_candidate_motion_restores_editing_after_terminal_verification(
    application, tmp_path, achieved_pose,
):
    from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
    from fault_detector_spot.application.controllers.command_controller import (
        CommandControllerState, CommandControllerStatus,
    )
    from fault_detector_spot.inspection.setup.probe_setup_motion import (
        ProbeMotionKind, ProbeMotionRequest,
    )
    from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
    from fault_detector_spot.ui.ros.command_state_order import CommandStateOrder
    from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient
    from fault_detector_spot.ui.ros.setup_state_order import SetupStateOrder
    from test_pre_approach_path import begin_setup
    from test_probe_setup_coordinator import coordinator

    probe, commands = coordinator(tmp_path)
    initial = begin_setup(probe, commands)
    controls = FinalizingInspectionControls(FakeUI())
    client = SimpleNamespace(
        client_id=initial.context.client_id,
        context_id=initial.context.context_id,
        _command_state_order=CommandStateOrder(),
        _state_order=SetupStateOrder(),
        _last_state_fingerprint=None,
        _action_request_ids={},
        _motion_goal_handles={},
        _finalization_goal_handles={},
        _capture_goal_handles={},
        _closing=False,
        state_changed=SimpleNamespace(emit=controls.apply_setup_state),
    )

    def message(snapshot, state_code):
        state = safe_state()
        state.client_id = snapshot.context.client_id
        state.context_id = snapshot.context.context_id
        state.revision = snapshot.context.revision
        state.state = state_code
        ProbeSetupStateAdapter._write_refinement(state, snapshot)
        ProbeSetupStateAdapter._write_approved_poses(state, snapshot.setup)
        return state

    assert ProbeSetupClient._emit_state(client, message(initial, ProbeSetupState.STATE_READY))
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    accepted = []

    def receive_status(status):
        state = message(status.snapshot, ProbeSetupMotionApi._public_state(status.state))
        state.request_id = status.request_id
        state.execution_instance_id = status.execution_instance_id
        state.status_sequence = status.status_sequence
        state.detail = status.detail
        accepted.append(ProbeSetupClient._emit_state(client, state))

    probe.add_motion_status_listener(receive_status)
    operation = probe.prepare_motion(
        initial.context, ProbeMotionRequest(ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH),
    )
    probe.submit_motion(operation)
    client._action_request_ids["candidate-goal"] = operation.request_id
    client._motion_goal_handles["candidate-goal"] = object()

    def publish_command_status(state, sequence):
        status = CommandControllerStatus(
            request=operation.request, state=state,
            execution_instance_id="candidate-execution", status_sequence=sequence,
        )
        for listener in tuple(commands.listeners):
            listener(status)

    publish_command_status(CommandControllerState.RUNNING, 1)
    assert accepted[-1]
    assert not controls.open_fine_adjustment_button.isEnabled()
    assert not dialog.add_pathing_point_button.isEnabled()
    assert not dialog.back_away_surface_button.isEnabled()
    assert not controls.use_current_alignment_button.isEnabled()

    target = probe._drafts[initial.context.context_id].refinement.pending_motion.target_pose_object
    probe.motion_state_source.pose = deepcopy(target)
    current_probe_pose = probe.motion_state_source.current_probe_pose_object
    if achieved_pose == "off_target":
        probe.motion_state_source.pose.position.x += .1
    elif achieved_pose == "unavailable":
        def unavailable(*_args):
            raise RuntimeError("No post-movement tag observation")
        probe.motion_state_source.current_probe_pose_object = unavailable

    publish_command_status(CommandControllerState.SUCCEEDED, 2)

    assert accepted[-1]
    assert controls._probe_setup_state.state == (
        ProbeSetupState.STATE_SUCCEEDED if achieved_pose == "reached"
        else ProbeSetupState.STATE_FAILED
    )
    assert controls._refinement_presentation.pending_motion is None
    assert not client._action_request_ids
    assert not client._motion_goal_handles
    if achieved_pose == "off_target":
        assert "Achieved probe position missed the target" in controls._probe_setup_state.detail
    elif achieved_pose == "unavailable":
        assert "No post-movement tag observation" in controls._probe_setup_state.detail
    assert controls.move_aligned_pose_button.isEnabled()
    assert controls.open_fine_adjustment_button.isEnabled()
    assert controls.orient_to_surface_button.isEnabled()
    assert controls.orient_to_tag_button.isEnabled()
    assert dialog.add_pathing_point_button.isEnabled()
    assert dialog.back_away_surface_button.isEnabled()
    assert controls.use_current_alignment_button.isEnabled()
    assert not dialog.next_button.isEnabled()

    probe.motion_state_source.current_probe_pose_object = current_probe_pose
    controls.use_current_alignment_button.click()
    assert controls.ui.requests[-1].operation == ProbeSetupIntent.OPERATION_APPROVE_ALIGNED_POSE
    approved = probe.approve_aligned_pose(
        probe.context(initial.context.context_id, initial.context.client_id),
    )
    assert ProbeSetupClient._emit_state(client, message(approved, ProbeSetupState.STATE_SUCCEEDED))
    assert dialog.next_button.isEnabled()
    dialog.next_button.click()
    assert dialog.workflow_stack.currentIndex() == dialog.PROBE_PAGE
    dialog.hide()
