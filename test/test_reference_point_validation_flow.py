"""Regression tests for saved-depth validation before refinement."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


class FakeProbeSetupClient:
    def __init__(self):
        self.context_id = ""


class FakeUI:
    def __init__(self):
        self.node = None
        self.status_label = QLabel()
        self.probe_setup_client = FakeProbeSetupClient()
        self.requests = []

    def execute_probe_setup(self, intent):
        self.requests.append(intent)
        return "request"

    def show_setup_unavailable(self, workflow):
        return False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def prepared_controls(valid_depth=True, valid_setup=True):
    controls = FinalizingInspectionControls(FakeUI())
    state = ProbeSetupState()
    state.selected_object_id = "object"
    state.selected_routine_id = "routine"
    state.reference_view_ids = ["slot6_hand"]
    state.reference_camera_ids = ["hand"]
    state.has_reference_pixel = True
    state.selected_reference_view_id = "slot6_hand"
    state.reference_pixel_u = 40
    state.reference_pixel_v = 50
    state.has_surface_point = valid_depth
    state.depth_m = 0.42
    state.has_probe_setup = valid_setup
    if not valid_depth:
        state.validation_error = "No valid depth near selected pixel"

    controls._preview_signature = (
        state.selected_object_id,
        state.selected_routine_id,
        tuple(state.reference_view_ids),
    )
    controls._probe_setup_state = state
    controls._reference_slot_view_ids[0] = "slot6_hand"

    class Point:
        u = 40
        v = 50

    controls.reference_view_widget._selected_image_point = Point()
    controls.refinement_dialog.open_reference_selection(True)
    controls.refinement_dialog.enable_reference_selection()
    return controls


def test_valid_saved_depth_enables_approval(application):
    controls = prepared_controls(valid_depth=True, valid_setup=True)

    controls.refinement_dialog.refresh_reference_selection()

    assert controls.refinement_dialog.approve_reference_button.isEnabled()
    assert controls.refinement_dialog.reference_depth_value_label.text() == (
        "0.420 m"
    )


def test_invalid_saved_depth_disables_approval_and_explains_why(application):
    controls = prepared_controls(valid_depth=False, valid_setup=False)

    controls.refinement_dialog.refresh_reference_selection()

    assert not controls.refinement_dialog.approve_reference_button.isEnabled()
    assert "No valid depth" in (
        controls.refinement_dialog.reference_depth_status_label.text()
    )


def test_approve_valid_point_starts_refinement_without_confirmation_wait(
    application,
):
    controls = prepared_controls(valid_depth=True, valid_setup=True)

    assert controls.handle_reference_point_approved() is True
    assert controls.refinement_dialog.workflow_stack.currentIndex() == controls.refinement_dialog.SAFE_APPROACH_PAGE
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    )
    assert "Confirming" not in (
        controls.refinement_dialog.reference_status_label.text()
    )


@pytest.mark.parametrize("status", [None, "pending", "none"])
def test_entry_requires_confirmed_attachment(application, status):
    from types import SimpleNamespace
    from fault_detector_spot.ui.sensor.models import SensorAttachmentViewStatus
    controls = prepared_controls()
    controls.refinement_dialog.hide()
    controls.ui._sensor_attachment_state = (
        None if status is None else SimpleNamespace(
            status=SensorAttachmentViewStatus(status)
        )
    )
    warnings = []
    controls.show_warning = lambda title, detail: warnings.append(detail)
    assert controls.handle_start_probe_refinement() is False
    assert not controls.refinement_dialog.isVisible()
    assert "Confirm the sensor attachment" in warnings[-1]
    assert controls.ui.requests == []


def test_start_failure_is_visible_and_retryable(application):
    controls = prepared_controls()
    assert controls.handle_reference_point_approved()
    assert not controls.refinement_dialog.approve_reference_button.isEnabled()
    assert not controls.handle_reference_point_approved()
    state = controls._probe_setup_state
    state.operation = ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    state.state = ProbeSetupState.STATE_FAILED
    state.detail = "No stable reference tag observation is available"
    controls.apply_setup_state(state)
    assert state.detail in controls.refinement_dialog.refinement_start_status_label.text()
    assert controls.refinement_dialog.workflow_stack.currentIndex() == controls.refinement_dialog.SAFE_APPROACH_PAGE
    assert not controls.move_calculated_approach_button.isEnabled()
    assert not controls.use_current_approach_button.isEnabled()
    assert controls.refinement_dialog.approve_reference_button.isEnabled()
    controls.refinement_dialog.retry_refinement_start_button.click()
    assert len(controls.ui.requests) == 2


def test_start_transport_failure_is_visible(application):
    controls = prepared_controls()
    controls.handle_reference_point_approved()
    controls.handle_reference_start_rejected("Service disconnected")
    assert "Service disconnected" in controls.refinement_dialog.refinement_start_status_label.text()
    assert controls.refinement_dialog.approve_reference_button.isEnabled()


def test_successful_start_opens_safe_approach_panel(application):
    controls = prepared_controls()
    controls.refinement_dialog.refresh_reference_selection()
    controls.refinement_dialog.approve_reference_button.click()
    assert controls.ui.requests[-1].operation == ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    state = controls._probe_setup_state
    state.operation = ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    state.state = ProbeSetupState.STATE_SUCCEEDED
    state.refinement_active = True
    state.refinement_stage = ProbeSetupState.REFINEMENT_STAGE_SAFE_APPROACH
    for name in ("safe_approach_candidate_pose_object", "aligned_preapproach_candidate_pose_object", "probe_candidate_pose_object"):
        getattr(state, name).orientation.w = 1.0
    controls.apply_setup_state(state)
    assert controls.refinement_dialog.workflow_stack.currentIndex() == controls.refinement_dialog.SAFE_APPROACH_PAGE
    assert not controls._reference_start_pending
