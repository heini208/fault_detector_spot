"""A successful wizard surface move unlocks summary and finalization."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent

from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_probe_point_guided_workflow import FakeUI, application
from test_probe_safe_approach_navigation import safe_state


@pytest.fixture
def wizard(application):
    ui = FakeUI()
    ui.application_client = SimpleNamespace(client_id="probe-ui")
    operations, finalizations = [], []
    ui.execute_operation = lambda intent, context_id="": (
        operations.append((intent, context_id)) or "local-operation"
    )
    ui.execute_probe_refinement_finalization = lambda **kwargs: (
        finalizations.append(kwargs) or "finalization"
    )
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.context_id = "probe-context"
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    state.surface_alignment_approved = True
    controls.apply_setup_state(state)
    controls.refinement_dialog.show_stage(RefinementStage.PROBE)
    controls.probe_point_id_field.setText("surface-point")
    controls.probe_point_display_name_field.setText("Surface point")
    controls.probe_position_tolerance_field.setText("0.01")
    controls.probe_orientation_tolerance_field.setText("0.05")
    controls.probe_measurement_duration_field.setText("1.0")
    controls._refresh_refinement_dialog()
    yield controls, state, operations, finalizations
    controls.refinement_dialog.hide()


def status(state=ApplicationCommandState.STATE_SUCCEEDED, **changes):
    result = ApplicationCommandState()
    result.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    result.context_id = "probe-context"
    result.client_id = "probe-ui"
    result.request_id = "server-operation"
    result.state = state
    for key, value in changes.items():
        setattr(result, key, value)
    return result


def test_success_enables_footer_next_and_save_on_summary(wizard):
    controls, state, operations, finalizations = wizard
    dialog = controls.refinement_dialog
    assert not dialog.next_button.isHidden()
    assert not dialog.next_button.isEnabled()
    assert controls.handle_test_surface_distance()
    intent, context_id = operations[-1]
    assert context_id == state.context_id
    assert intent.intent == OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    assert intent.target_surface_distance_m == pytest.approx(0.1)

    controls.handle_application_state(status())
    # Subsequent authoritative snapshots must not hide/reset successful navigation.
    controls.apply_setup_state(state)
    assert dialog.next_button.isEnabled()
    assert dialog.continue_to_summary_button.isEnabled()
    assert controls.handle_refinement_next()
    assert dialog.is_summary_page()
    assert "Step 5 of 5" in dialog.progress_label.text()
    assert dialog.next_button.isHidden()
    assert "Calculated from the approved alignment" in dialog.summary_final_pose_label.text()
    assert controls.approve_and_retract_button.isEnabled()
    controls.approve_and_retract_button.click()
    assert finalizations == [{
        "save_requested": True,
        "probe_point_id": "surface-point",
        "probe_point_display_name": "Surface point",
        "position_tolerance_m": 0.01,
        "orientation_tolerance_rad": 0.05,
        "measurement_duration_sec": 1.0,
    }]


@pytest.mark.parametrize("result", [
    ApplicationCommandState.STATE_RUNNING,
    ApplicationCommandState.STATE_FAILED,
    ApplicationCommandState.STATE_CANCELLED,
])
def test_unfinished_or_failed_surface_move_cannot_advance(wizard, result):
    controls, *_ = wizard
    assert controls.handle_test_surface_distance()
    controls.handle_application_state(status(result))
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
    assert not controls.approve_and_retract_button.isEnabled()


@pytest.mark.parametrize("changes", [
    {"context_id": ""}, {"context_id": "another-setup"},
    {"client_id": "another-ui"},
])
def test_other_operation_success_does_not_complete_this_wizard(wizard, changes):
    controls, *_ = wizard
    controls.handle_test_surface_distance()
    controls.handle_application_state(status(**changes))
    assert controls._surface_test_active
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()


@pytest.mark.parametrize("change", ["target", "context", "external_motion", "recovery"])
def test_changed_setup_or_recovery_invalidates_summary_admission(wizard, change):
    controls, state, *_ = wizard
    controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    if change == "target":
        state.target_surface_distance_m = 0.12
    elif change == "context":
        state.context_id = "another-setup"
    elif change == "external_motion":
        state.alignment_motion_state = state.MOTION_NOT_TESTED
    else:
        state.refinement_recovery_required = True
    controls.apply_setup_state(state)
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
    assert not controls.approve_and_retract_button.isEnabled()


def test_new_surface_attempt_disables_previously_enabled_next(wizard):
    controls, _, operations, _ = wizard
    controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    assert controls.refinement_dialog.next_button.isEnabled()
    assert controls.handle_test_surface_distance()
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert len(operations) == 2


def test_rejected_new_surface_attempt_keeps_next_disabled(wizard):
    controls, *_ = wizard
    controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    assert controls.refinement_dialog.next_button.isEnabled()
    controls.ui.execute_operation = lambda *args, **kwargs: None
    assert not controls.handle_test_surface_distance()
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
