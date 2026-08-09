"""Tests for the single approved probe-point persistence path."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)


class FakePublisher:
    def publish(self, message):
        pass


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


def test_hidden_direct_save_path_is_removed(tmp_path):
    application = QApplication.instance() or QApplication([])
    controls = InspectionControls(FakeUI(tmp_path))

    assert application is not None
    assert not hasattr(controls, "save_probe_point_button")
    assert not hasattr(controls, "handle_save_probe_point")
    assert hasattr(controls, "handle_approve_and_retract")
