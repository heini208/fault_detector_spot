"""Tests for remote reference previews in inspection controls."""

import os
from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import ProbeSetupState
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import Image

from fault_detector_spot.ui.inspection.controls import InspectionControls


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class FakeProbeSetupClient:
    def __init__(self):
        self.context_id = "probe-context"
        self.preview_requests = []

    def request_preview(self, view_id):
        self.preview_requests.append(view_id)
        return SimpleNamespace()


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


def image():
    message = Image()
    message.width = 4
    message.height = 3
    message.encoding = "rgb8"
    message.step = 12
    message.data = bytes([0] * 36)
    return message


def setup_state():
    state = ProbeSetupState()
    state.context_id = "probe-context"
    state.state = ProbeSetupState.STATE_READY
    state.object_ids = ["motor"]
    state.selected_object_id = "motor"
    state.routine_ids = ["scan"]
    state.selected_routine_id = "scan"
    state.reference_view_ids = ["slot1_hand"]
    state.reference_camera_ids = ["hand"]
    state.selected_reference_view_id = "slot1_hand"
    state.has_reference_pixel = True
    state.reference_pixel_u = 2
    state.reference_pixel_v = 1
    return state


def preview(view_id="slot1_hand"):
    return SimpleNamespace(
        reference_view_id=view_id,
        camera_id="hand",
        slot_index=0,
        image=image(),
        selectable_x=0,
        selectable_y=0,
        selectable_width=4,
        selectable_height=3,
    )


def test_state_requests_preview_from_application_service(application):
    ui = FakeUI()
    controls = InspectionControls(ui)
    ui.probe_setup_client.preview_requests.clear()

    controls.apply_setup_state(setup_state())

    assert ui.probe_setup_client.preview_requests == ["slot1_hand"]


def test_preview_preserves_authoritative_source_pixel(application):
    controls = InspectionControls(FakeUI())
    controls.apply_setup_state(setup_state())

    assert controls.apply_reference_preview(preview()) is True

    widget = controls.reference_view_widgets[0]
    assert widget.selected_image_point.u == 2
    assert widget.selected_image_point.v == 1
    assert controls._reference_slot_view_ids[0] == "slot1_hand"


def test_stale_preview_is_ignored(application):
    controls = InspectionControls(FakeUI())
    controls.apply_setup_state(setup_state())

    assert controls.apply_reference_preview(preview("old_view")) is False
    assert controls._reference_slot_view_ids == ["", "", ""]
