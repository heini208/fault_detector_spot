"""Tests for inspection routine sensor decoupling."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.finalizing_controls import (
    FinalizingInspectionControls,
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


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_routine_creation_readiness_depends_only_on_parent(
    application,
    tmp_path,
):
    controls = FinalizingInspectionControls(FakeUI(tmp_path))

    assert controls.create_routine_button.isEnabled() is False
    controls.routine_parent_object_dropdown.addItem(
        "Select existing object",
        None,
    )
    controls.routine_parent_object_dropdown.addItem("motor", "motor")
    controls.routine_parent_object_dropdown.setCurrentIndex(1)

    assert controls.create_routine_button.isEnabled() is True
