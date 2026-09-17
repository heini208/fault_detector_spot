"""Tests for the first single-view reference preview UI pass."""

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


def make_preview(view_id, camera_id):
    image = Image()
    image.width = 4
    image.height = 3
    image.encoding = "rgb8"
    image.step = 12
    image.data = bytes([0] * 36)
    return SimpleNamespace(
        reference_view_id=view_id,
        camera_id=camera_id,
        image=image,
        selectable_x=0,
        selectable_y=0,
        selectable_width=4,
        selectable_height=3,
    )


def test_preloads_all_six_reference_previews(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)

    controls.apply_setup_state(make_state())

    assert ui.probe_setup_client.preview_requests == [
        "slot1_frontleft",
        "slot2_frontright",
        "slot3_left",
        "slot4_right",
        "slot5_back",
        "slot6_hand",
    ]


def test_switching_camera_uses_cached_preview(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls.apply_setup_state(make_state())
    controls.apply_reference_preview(
        make_preview("slot6_hand", "hand")
    )
    controls.apply_reference_preview(
        make_preview("slot5_back", "back")
    )
    request_count = len(ui.probe_setup_client.preview_requests)

    dropdown = controls.reference_camera_dropdowns[0]
    dropdown.setCurrentIndex(dropdown.findData("back"))

    assert len(ui.probe_setup_client.preview_requests) == request_count
    assert controls._reference_slot_view_ids[0] == "slot5_back"


def test_only_one_reference_viewer_is_left_visible(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)

    assert controls.reference_view_widgets[1].isHidden()
    assert controls.reference_view_widgets[2].isHidden()
    assert controls.reference_camera_dropdowns[1].isHidden()
    assert controls.reference_camera_dropdowns[2].isHidden()
