"""Tests for inspection routine sensor decoupling."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.controls import (
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


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_routine_creation_readiness_depends_only_on_parent(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))

    assert controls.create_routine_button.isEnabled() is False
    controls.routine_parent_object_dropdown.addItem(
        "Select existing object",
        None,
    )
    controls.routine_parent_object_dropdown.addItem("motor", "motor")
    controls.routine_parent_object_dropdown.setCurrentIndex(1)

    assert controls.create_routine_button.isEnabled() is True


def test_inspection_controls_do_not_own_sensor_registry_mutations(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))

    forbidden = (
        "sensor_add_client",
        "sensor_list_subscription",
        "sensor_id_field",
        "probe_frame_value_label",
        "set_sensor_definitions",
        "_populate_sensor_dropdown",
        "_active_probe_frame",
        "_configured_hand_to_probe_pose",
        "new_sensor_id_field",
        "handle_add_sensor",
    )
    for name in forbidden:
        assert not hasattr(controls, name)
