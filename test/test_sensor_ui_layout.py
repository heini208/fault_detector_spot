"""Tests for the presentation-only sensor mount UI."""

import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.sensor.controls import SensorControls


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_sensor_workspace_exposes_expected_visual_sections(application):
    controls = SensorControls()

    assert controls.active_mount_dropdown.isEnabled() is False
    assert controls.select_mount_button.isEnabled() is False
    assert controls.confirm_attachment_button.isEnabled() is False
    assert controls.clear_attachment_button.isEnabled() is False
    assert controls.new_mount_button.isEnabled() is False
    assert controls.retire_mount_button.isEnabled() is False
    assert controls.save_mount_button.isEnabled() is False
    assert controls.start_configuration_button.isEnabled() is False
    assert controls.manage_sensing_elements_button.isEnabled() is False
    assert controls.mount_table.columnCount() == 4


def test_sensor_workspace_preview_rows_are_presentation_only(application):
    controls = SensorControls()

    controls.add_preview_mount(
        "Hall probe mount",
        "bmm150_mount",
        "bmm150_mount_probe",
        "Calibrated",
    )

    assert controls.mount_table.rowCount() == 1
    assert controls.mount_table.item(0, 0).text() == "Hall probe mount"
    assert controls.mount_table.item(0, 1).text() == "bmm150_mount"
    assert controls.empty_registry_label.isHidden()


def test_sensor_workspace_has_no_ros_or_backend_ownership():
    path = (
        Path(__file__).parents[1]
        / "fault_detector_spot"
        / "ui"
        / "sensor"
        / "controls.py"
    )
    source = path.read_text(encoding="utf-8")

    forbidden = (
        "rclpy",
        "fault_detector_msgs",
        "SensorRepository",
        "SensorAttachmentController",
        "CommandController",
        "create_subscription",
        "create_client",
        "call_async",
    )

    for value in forbidden:
        assert value not in source
