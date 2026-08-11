"""Tests for registry-backed inspection sensor controls."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.data.models import PoseData
from fault_detector_spot.inspection.data.sensor_models import SensorDefinition


class FakePublisher:
    def publish(self, message):
        pass


class FakeFuture:
    def __init__(self, response):
        self.response = response

    def result(self):
        return self.response

    def add_done_callback(self, callback):
        callback(self)


class FakeClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.response)


class FakeUI:
    def __init__(self, object_root, sensors=None):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}
        self.sensor_definitions = list(sensors or [])

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


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


def test_add_sensor_submits_normalized_calibration_request(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.show_warning = lambda title, message: None
    controls.new_sensor_id_field.setText("bmm150_01")
    controls.new_sensor_display_name_field.setText("BMM150 Hall sensor")
    controls.sensor_translation_fields[0].setText("0.20")
    controls.sensor_rotation_fields[2].setText("90.0")
    response = SimpleNamespace(success=True, message="created")
    client = FakeClient(response)
    controls.sensor_add_client = client

    assert controls.handle_add_sensor() is True

    request = client.requests[0]
    assert request.sensor.sensor_id == "bmm150_01"
    assert request.sensor.hand_to_probe.position.x == pytest.approx(0.20)
    assert request.sensor.hand_to_probe.orientation.z == pytest.approx(
        2 ** -0.5
    )
    assert request.sensor.hand_to_probe.orientation.w == pytest.approx(
        2 ** -0.5
    )
    assert controls.sensor_creation_status_label.text() == "created"


def test_retire_sensor_uses_registry_and_removes_local_selection(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path, [sensor()]))
    controls.show_warning = lambda title, message: None
    controls.ask_question = lambda title, message: True
    response = SimpleNamespace(
        success=True,
        message="retired; restart required",
    )
    client = FakeClient(response)
    controls.sensor_retire_client = client
    index = controls.retire_sensor_dropdown.findData("bmm150_01")
    controls.retire_sensor_dropdown.setCurrentIndex(index)

    assert controls.handle_retire_sensor() is True

    assert client.requests[0].sensor_id == "bmm150_01"
    assert "bmm150_01" not in controls._sensor_definitions
    assert controls.retire_sensor_dropdown.findData("bmm150_01") == -1
    assert controls.sensor_id_field.findData("bmm150_01") == -1
    assert controls.sensor_creation_status_label.text() == (
        "retired; restart required"
    )
