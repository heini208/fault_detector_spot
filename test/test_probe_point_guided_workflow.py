"""Tests for the guided add-probe-point UI workflow."""

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
        self.preview_requests = []

    def request_preview(self, view_id):
        self.preview_requests.append(view_id)
        return object()


class FakeUI:
    def __init__(self):
        self.node = None
        self.status_label = QLabel()
        self.probe_setup_client = FakeProbeSetupClient()
        self.capture_calls = []
        self.requests = []

    def execute_probe_reference_capture(
        self,
        camera_ids,
        replace_existing=False,
    ):
        self.capture_calls.append((tuple(camera_ids), replace_existing))
        return "capture-request"

    def execute_probe_setup(self, intent):
        self.requests.append(intent)
        return "probe-setup-request"

    def show_setup_unavailable(self, workflow):
        return False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def make_state(with_references=True):
    state = ProbeSetupState()
    state.context_id = "probe-context"
    state.state = ProbeSetupState.STATE_READY
    state.object_ids = ["motor"]
    state.selected_object_id = "motor"
    state.routine_ids = ["scan"]
    state.selected_routine_id = "scan"
    if with_references:
        state.reference_view_ids = [
            "slot1_frontleft",
            "slot2_frontright",
            "slot3_left",
            "slot4_right",
            "slot5_back",
            "slot6_hand",
        ]
        state.reference_camera_ids = [
            "frontleft",
            "frontright",
            "left",
            "right",
            "back",
            "hand",
        ]
    return state


def test_main_workspace_is_reduced_to_add_probe_point(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)

    assert controls.start_probe_refinement_button.text() == (
        "Add New Probe Point"
    )
    assert controls._probe_point_entry_panel is not None
    assert controls.inspection_workspace_splitter.widget(0).isHidden()
    assert controls.inspection_workspace_splitter.widget(1).isHidden()


def test_add_probe_point_opens_reference_step_before_refinement(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls._probe_setup_state = make_state(with_references=True)

    assert controls.handle_start_probe_refinement() is True

    assert (
        controls.refinement_dialog.workflow_stack.currentIndex()
        == controls.refinement_dialog.REFERENCE_PAGE
    )
    assert ui.requests == []
    assert controls.refinement_dialog.use_existing_reference_button.isVisible()
    assert controls.refinement_dialog.retake_reference_button.isVisible()


def test_workflow_contains_reference_three_motion_stages_and_summary(
    application,
):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)

    assert controls.refinement_dialog.workflow_stack.count() == 5
    assert controls.refinement_dialog.SAFE_APPROACH_PAGE == 1
    assert controls.refinement_dialog.ALIGNMENT_PAGE == 2
    assert controls.refinement_dialog.PROBE_PAGE == 3
    assert controls.refinement_dialog.SUMMARY_PAGE == 4
    assert controls.refinement_dialog.summary_safe_pose_label is not None
    assert controls.refinement_dialog.summary_aligned_pose_label is not None
    assert controls.probe_point_id_field.parent() is not None
    assert controls.probe_point_display_name_field.parent() is not None


def test_reference_page_offers_capture_when_no_saved_views(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls._probe_setup_state = make_state(with_references=False)

    controls.handle_start_probe_refinement()

    assert controls.refinement_dialog.capture_reference_button.isVisible()
    assert not controls.refinement_dialog.use_existing_reference_button.isVisible()
    assert not controls.refinement_dialog.retake_reference_button.isVisible()


def test_reference_approval_starts_existing_refinement_operation(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    state = make_state(with_references=True)
    state.has_reference_pixel = True
    state.selected_reference_view_id = "slot6_hand"
    state.reference_pixel_u = 40
    state.reference_pixel_v = 50
    controls._probe_setup_state = state
    controls._reference_slot_view_ids[0] = "slot6_hand"

    class Point:
        u = 40
        v = 50

    controls.reference_view_widget._selected_image_point = Point()

    assert controls.handle_reference_point_approved() is True
    assert ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    )
