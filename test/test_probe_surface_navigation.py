"""A successful wizard surface move unlocks summary and finalization."""

from types import SimpleNamespace
from concurrent.futures import Future
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent

from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
import fault_detector_spot.ui.ros.probe_setup_client as client_module
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


@pytest.mark.parametrize("rejected", [True, False], ids=["rejected", "uncertain"])
def test_only_confirmed_finalization_rejection_unlocks_saving(wizard, monkeypatch, rejected):
    controls, state, *_ = wizard
    goal_response = Future()
    transport = Mock()
    transport.send_goal_async.return_value = goal_response
    monkeypatch.setattr(client_module, "ActionClient", Mock(return_value=transport))
    client = client_module.ProbeSetupClient(Mock(), "probe-ui")
    client.context_id = state.context_id
    controls.ui.execute_probe_refinement_finalization = client.finalize_refinement
    harness = SimpleNamespace(inspection_controls=controls)
    client.finalization_rejected.connect(
        lambda detail: Fault_Detector_UI._process_probe_finalization_rejected(harness, detail)
    )
    client.request_rejected.connect(
        lambda detail: Fault_Detector_UI._process_probe_setup_rejected(harness, detail)
    )
    controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    assert controls.handle_refinement_next()
    assert controls.handle_approve_and_retract()
    client.request_rejected.emit("Unrelated setup request rejected")
    assert controls._probe_finalization_point_id == "surface-point"

    if rejected:
        goal_response.set_result(SimpleNamespace(accepted=False))
        assert controls._probe_finalization_point_id == ""
        assert controls._probe_finalization_scope is None
        assert "rejected" in controls.refinement_recovery_status_label.text()
    else:
        goal_response.set_exception(RuntimeError("Response lost"))
        assert controls._probe_finalization_point_id == "surface-point"
    assert controls.approve_and_retract_button.isEnabled() is rejected
    assert controls.refinement_dialog.close_button.isEnabled() is rejected
    assert controls.request_close_refinement_workflow() is rejected


@pytest.mark.parametrize("fully_custom", [False, True])
def test_finalization_keeps_close_and_abort_disabled_after_refresh(wizard, fully_custom):
    controls, state, *_ = wizard
    controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    state.fully_custom = fully_custom
    state.has_final_probe_candidate = True
    state.probe_pose_approved = True
    state.probe_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    assert controls.handle_refinement_next()
    assert controls.handle_approve_and_retract()

    # Repeated rendering and authoritative updates must agree with the handlers.
    for update in (controls._refresh_refinement_dialog, lambda: controls.apply_setup_state(state)):
        update()
        assert not controls.refinement_dialog.close_button.isEnabled()
        assert not controls.refinement_dialog.abort_button.isEnabled()
        assert not controls.abort_probe_refinement_button.isEnabled()
    assert not controls.request_close_refinement_workflow()
    assert not controls.handle_abort_probe_refinement()

    state.refinement_active = False
    controls.apply_setup_state(state)
    controls.refinement_dialog.open_mode_selection()
    assert controls.refinement_dialog.abort_button.isEnabled()
    assert controls.request_close_refinement_workflow()


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


@pytest.mark.parametrize("completed_before_back", [True, False])
def test_leaving_probe_stage_invalidates_surface_success(wizard, completed_before_back):
    controls, *_ = wizard
    assert controls.handle_test_surface_distance()
    if completed_before_back:
        controls.handle_application_state(status())

    assert controls.handle_refinement_back()
    if not completed_before_back:
        controls.handle_application_state(status())

    controls.refinement_dialog.show_stage(RefinementStage.PROBE)
    assert not controls.refinement_dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
    assert not controls.approve_and_retract_button.isEnabled()


@pytest.mark.parametrize("context_change", [
    "replaced", "closed", "replaced_custom", "closed_custom",
])
def test_setup_change_clears_measurement_without_refresh(wizard, monkeypatch, context_change):
    controls, state, *_ = wizard
    assert controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    assert controls.handle_refinement_back()
    assert controls.handle_test_sensor_tip_distance()
    feedback = status(
        ApplicationCommandState.STATE_RUNNING,
        intent=OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE,
        context_id=controls._sensor_tip_distance_context_id,
    )
    controls.handle_application_state(feedback)
    assert controls._sensor_tip_distance_request_id

    # Snapshot handling must discard stale correlation before rendering runs.
    monkeypatch.setattr(controls, "_refresh_refinement_dialog", lambda: None)
    state.fully_custom = context_change.endswith("_custom")
    if context_change.startswith("replaced"):
        state.context_id = "another-setup"
    else:
        state.refinement_active = False
    controls.apply_setup_state(state)

    assert not controls._sensor_tip_distance_test_active
    assert controls._sensor_tip_distance_context_id == ""
    assert controls._sensor_tip_distance_request_id == ""
    assert not controls._surface_test_active
    assert not controls._surface_move_succeeded
    assert controls._surface_move_target_m is None
    assert controls._surface_move_context_id == ""

    feedback.state = ApplicationCommandState.STATE_FAILED
    controls.handle_application_state(feedback)
    assert not controls._distance_failure_requires_retraction


def test_replaced_setup_ignores_late_surface_cancellation(wizard):
    controls, state, *_ = wizard
    assert controls.handle_test_surface_distance()
    state.context_id = "another-setup"
    controls.apply_setup_state(state)

    controls.handle_application_state(status(ApplicationCommandState.STATE_CANCELLED))

    assert not controls._distance_failure_requires_retraction
    assert not controls._surface_test_active
    assert controls._surface_move_context_id == ""


def test_pause_and_resume_preserve_successful_surface_move(wizard):
    controls, *_ = wizard
    assert controls.handle_test_surface_distance()
    controls.handle_application_state(status())
    presentation = controls._refinement_presentation
    dialog = controls.refinement_dialog
    assert dialog.next_button.isEnabled()

    assert dialog.close()
    assert not dialog.isVisible()
    assert controls.resume_refinement_dialog()

    assert controls._refinement_presentation is presentation
    assert controls._surface_move_succeeded
    assert dialog.isVisible()
    assert dialog.next_button.isEnabled()
    assert controls.handle_refinement_next()
    assert dialog.is_summary_page()
