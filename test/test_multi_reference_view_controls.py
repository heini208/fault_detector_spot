"""Tests for selecting cameras in the inspection setup UI."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeUI:
    def __init__(self, object_root):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def configure_required_fields(controls):
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")


def test_capture_command_preserves_three_camera_slots(application, tmp_path):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    configure_required_fields(controls)
    selections = ("frontleft", "", "hand")
    for dropdown, camera_id in zip(
        controls.reference_camera_dropdowns,
        selections,
    ):
        dropdown.setCurrentIndex(dropdown.findData(camera_id))

    assert controls.handle_capture_reference_view()

    message = ui.complex_command_publisher.messages[-1]
    assert message.command.command_id == (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
    )
    assert list(message.inspection.reference_camera_ids) == list(selections)


def test_duplicate_camera_selection_is_rejected(application, tmp_path):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    configure_required_fields(controls)
    for dropdown in controls.reference_camera_dropdowns[:2]:
        dropdown.setCurrentIndex(dropdown.findData("hand"))
    warnings = []
    controls.show_warning = lambda title, message: warnings.append(
        (title, message)
    )

    assert not controls.handle_capture_reference_view()
    assert warnings[0][0] == "Duplicate Camera"
    assert ui.complex_command_publisher.messages == []
