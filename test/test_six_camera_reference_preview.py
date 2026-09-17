"""Tests for six-camera capture with three preview selectors."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import ProbeSetupState
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import Image

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
)


class FakeProbeSetupClient:
    def __init__(self):
        self.context_id = "probe-context"
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
        return "request"

    def show_setup_unavailable(self, workflow):
        return False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def make_state():
    state = ProbeSetupState()
    state.context_id = "probe-context"
    state.state = ProbeSetupState.STATE_READY
    state.object_ids = ["motor"]
    state.selected_object_id = "motor"
    state.routine_ids = ["scan"]
    state.selected_routine_id = "scan"
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


def make_preview(view_id, camera_id, slot_index):
    image = Image()
    image.width = 4
    image.height = 3
    image.encoding = "rgb8"
    image.step = 12
    image.data = bytes([0] * 36)
    return SimpleNamespace(
        reference_view_id=view_id,
        camera_id=camera_id,
        slot_index=slot_index,
        image=image,
        selectable_x=0,
        selectable_y=0,
        selectable_width=4,
        selectable_height=3,
    )


def test_capture_request_is_independent_of_display_dropdowns(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls._probe_setup_state = make_state()

    for index, camera_id in enumerate(("hand", "back", "right")):
        dropdown = controls.reference_camera_dropdowns[index]
        dropdown.setCurrentIndex(dropdown.findData(camera_id))

    assert controls.handle_capture_reference_view() is True
    assert ui.capture_calls == [(
        ("frontleft", "frontright", "left"),
        False,
    )]


def test_dropdown_fetches_saved_camera_from_any_capture_slot(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls.apply_setup_state(make_state())
    ui.probe_setup_client.preview_requests.clear()

    dropdown = controls.reference_camera_dropdowns[1]
    dropdown.setCurrentIndex(dropdown.findData("right"))

    assert ui.probe_setup_client.preview_requests == ["slot4_right"]

    response = make_preview("slot4_right", "right", 3)
    assert controls.apply_reference_preview(response) is True
    assert controls._reference_slot_view_ids[1] == "slot4_right"


def test_preview_is_routed_by_camera_not_persisted_slot(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls.apply_setup_state(make_state())
    ui.probe_setup_client.preview_requests.clear()

    dropdown = controls.reference_camera_dropdowns[2]
    dropdown.setCurrentIndex(dropdown.findData("hand"))

    response = make_preview("slot6_hand", "hand", 5)
    assert controls.apply_reference_preview(response) is True
    assert controls._reference_slot_view_ids[2] == "slot6_hand"
