"""Safe-pose wizard navigation consumes authoritative motion completion."""

import pytest
from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_probe_point_guided_workflow import FakeUI, application
from test_probe_setup_state_adapter import complete_state, pose


def safe_state():
    state = complete_state()
    state.object_ids = ["motor"]
    state.selected_object_id = "motor"
    state.routine_ids = ["scan"]
    state.selected_routine_id = "scan"
    state.has_routine_safe_approach_pose = True
    state.refinement_active = True
    state.refinement_stage = state.REFINEMENT_STAGE_SAFE_APPROACH
    pose(state.safe_approach_candidate_pose_object, 0.80)
    pose(state.aligned_preapproach_candidate_pose_object, 0.60)
    pose(state.probe_candidate_pose_object, 0.50)
    return state


def test_reached_safe_pose_enables_continue_without_per_point_approval(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    assert not dialog.next_button.isEnabled()
    assert dialog.fine_adjustment_dialog.isHidden()
    state.motion_pending = True
    state.safe_approach_motion_state = state.MOTION_MOVING
    controls.apply_setup_state(state)
    assert not dialog.next_button.isEnabled()
    state.motion_pending = False
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.state = state.STATE_SUCCEEDED
    controls.apply_setup_state(state)
    assert dialog.next_button.isEnabled()
    dialog.next_button.click()
    assert dialog.workflow_stack.currentIndex() == dialog.ALIGNMENT_PAGE
    assert dialog.fine_adjustment_dialog.isHidden()
    controls.refinement_dialog.hide()


@pytest.mark.parametrize("motion_state", [
    ProbeSetupState.MOTION_NOT_TESTED,
    ProbeSetupState.MOTION_MOVING,
    ProbeSetupState.MOTION_FAILED,
])
def test_safe_step_does_not_advance_without_verified_completion(application, motion_state):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = motion_state
    controls.apply_setup_state(state)
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
    controls.refinement_dialog.hide()


@pytest.mark.parametrize("pose_available", [True, False])
def test_safe_command_success_allows_continue(application, tmp_path, pose_available):
    from fault_detector_spot.inspection.model.models import ImagePoint
    from fault_detector_spot.inspection.setup.probe_setup_motion import (
        ProbeMotionKind, ProbeMotionRequest,
    )
    from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
        ProbeSetupStateAdapter,
    )
    from test_probe_setup_coordinator import coordinator, create_selected_routine

    probe, commands = coordinator(tmp_path)
    snapshot = create_selected_routine(probe, probe.open_context("probe-ui").context)
    snapshot = probe.select_reference_pixel(
        snapshot.context, "slot1_hand", ImagePoint(u=20, v=30),
        "surface_fit", 0.10, 0.20,
    )
    snapshot = probe.begin_refinement(snapshot.context)
    operation = probe.prepare_motion(
        snapshot.context, ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
    )
    probe.submit_motion(operation)
    achieved = snapshot.refinement.candidate_pose(RefinementStage.SAFE_APPROACH)
    achieved.position.x += 0.0171
    probe.motion_state_source.pose = achieved
    if not pose_available:
        def unavailable(*_args):
            raise RuntimeError("No post-movement tag observation")
        probe.motion_state_source.current_probe_pose_object = unavailable
    pending = probe._drafts[snapshot.context.context_id].refinement.pending_motion
    assert not pending.verify_achieved_pose
    assert not pending.updates_candidate
    commands.succeed(operation.request)
    completed = probe.snapshot(probe.context(snapshot.context.context_id, "probe-ui"))

    state = safe_state()
    ProbeSetupStateAdapter._write_refinement(state, completed)
    ProbeSetupStateAdapter._write_approved_poses(state, completed.setup)
    controls = FinalizingInspectionControls(FakeUI())
    controls.apply_setup_state(state)
    assert controls.refinement_dialog.next_button.isEnabled()
    controls.refinement_dialog.next_button.click()
    assert controls.refinement_dialog.workflow_stack.currentIndex() == (
        controls.refinement_dialog.ALIGNMENT_PAGE
    )
    controls.refinement_dialog.hide()
