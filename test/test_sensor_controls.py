"""Tests for inspection routine sensor presentation."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.model.models import PoseData
from fault_detector_spot.inspection.model.sensor_models import SensorDefinition


class FakePublisher:
    def publish(self, message):
        pass


class FakeUI:
    def __init__(self, object_root, sensors=None):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}
        self.sensor_definitions = list(sensors or [])


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def sensor(sensor_id="bmm150_01"):
    return SensorDefinition(
        sensor_id=sensor_id,
        display_name="BMM150 Hall sensor",
        hand_to_probe=PoseData.identity(),
    )


def test_sensor_dropdown_derives_probe_frame(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path, [sensor()]))

    index = controls.sensor_id_field.findData("bmm150_01")
    controls.sensor_id_field.setCurrentIndex(index)

    assert controls.sensor_id_field.currentData() == "bmm150_01"
    assert controls.probe_frame_value_label.text() == (
        "bmm150_01_probe"
    )
    assert controls._active_probe_frame() == "bmm150_01_probe"


def test_unavailable_saved_sensor_is_visible_but_not_registered(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))

    controls._populate_sensor_dropdown("missing_mount")

    assert controls.sensor_id_field.currentData() == "missing_mount"
    assert "sensor unavailable" in controls.probe_frame_value_label.text()
    assert controls.create_routine_button.isEnabled() is False


def test_global_definition_refresh_preserves_routine_selection(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path, [sensor()]))
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("bmm150_01")
    )
    controls.set_sensor_definitions(
        [sensor(), sensor("replacement_mount")]
    )

    assert controls.sensor_id_field.currentData() == "bmm150_01"
    assert controls.sensor_id_field.findData("replacement_mount") >= 0
    assert controls.probe_frame_value_label.text() == "bmm150_01_probe"


def test_inspection_controls_do_not_own_sensor_registry_mutations(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path, [sensor()]))

    forbidden = (
        "sensor_add_client",
        "sensor_retire_client",
        "sensor_list_subscription",
        "new_sensor_id_field",
        "retire_sensor_dropdown",
        "handle_add_sensor",
        "handle_retire_sensor",
    )
    for name in forbidden:
        assert not hasattr(controls, name)
