"""Tests for the presentation-only sensor mount UI."""

import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.sensor.controls import SensorControls


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def definition(sensor_id, display_name):
    return SimpleNamespace(
        sensor_id=sensor_id,
        display_name=display_name,
        probe_frame=f"{sensor_id}_probe",
    )


def attachment(
    status,
    active_sensor_id="",
    pending_sensor_id="",
    attachment_revision=0,
):
    return SimpleNamespace(
        status=SimpleNamespace(value=status),
        active_sensor_id=active_sensor_id,
        pending_sensor_id=pending_sensor_id,
        attachment_revision=attachment_revision,
        selected_sensor_id=pending_sensor_id or active_sensor_id,
    )


def test_sensor_workspace_waits_for_authoritative_registry(application):
    controls = SensorControls()

    assert controls.active_mount_dropdown.isEnabled() is False
    assert controls.select_mount_button.isEnabled() is False
    assert controls.clear_attachment_button.isEnabled() is False
    assert controls.new_mount_button.isEnabled() is False
    assert controls.retire_mount_button.isEnabled() is False
    assert controls.save_mount_button.isEnabled() is False
    assert controls.start_configuration_button.isEnabled() is False
    assert controls.mount_table.columnCount() == 4
    assert controls.mount_table.rowCount() == 0


def test_registry_contains_only_real_physical_sensors(application):
    controls = SensorControls()

    controls.apply_definitions((
        definition("hall_probe", "Hall probe"),
    ))

    assert controls.mount_table.rowCount() == 1
    assert controls.mount_table.item(0, 0).text() == "Hall probe"
    assert controls.mount_table.item(0, 1).text() == "hall_probe"
    assert controls.mount_table.item(0, 2).text() == "hall_probe_probe"
    assert controls.mount_table.item(0, 3).text() == "Calibrated"
    assert controls.active_mount_dropdown.count() == 1
    assert controls.active_mount_dropdown.itemData(0) == "hall_probe"
    assert controls.clear_attachment_button.text() == "Remove Sensor"


def test_no_sensor_state_uses_hand_without_registry_entry(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.apply_attachment_state(
        attachment("none", attachment_revision=4)
    )

    assert controls.mount_table.rowCount() == 0
    assert controls.active_mount_dropdown.isEnabled() is False
    assert controls.attachment_name_value.text() == "No sensor"
    assert controls.attachment_status_value.text() == "Confirmation pending"
    assert controls.attachment_probe_frame_value.text() == "hand"
    assert controls.attachment_revision_value.text() == "4"
    assert controls.clear_attachment_button.isEnabled() is False


def test_pending_attachment_is_presented_without_local_confirm_button(
    application,
):
    controls = SensorControls()
    controls.apply_definitions((
        definition("hall_probe", "Hall probe"),
    ))

    controls.apply_attachment_state(
        attachment(
            "pending",
            pending_sensor_id="hall_probe",
            attachment_revision=7,
        )
    )

    assert controls.attachment_name_value.text() == "Hall probe"
    assert controls.attachment_status_value.text() == "Confirmation pending"
    assert not hasattr(controls, "confirm_attachment_button")


def test_remove_sensor_emits_empty_selection(application):
    controls = SensorControls()
    controls.apply_definitions((
        definition("hall_probe", "Hall probe"),
    ))
    controls.apply_attachment_state(
        attachment(
            "active",
            active_sensor_id="hall_probe",
            attachment_revision=8,
        )
    )
    selections = []
    controls.select_requested.connect(selections.append)

    controls.clear_attachment_button.click()

    assert selections == [""]


def test_sensor_workspace_has_no_sensing_elements_placeholder(application):
    controls = SensorControls()

    assert not hasattr(controls, "manage_sensing_elements_button")


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


def test_main_ui_contains_sensor_mount_tab_and_attachment_client():
    path = (
        Path(__file__).parents[1]
        / "fault_detector_spot"
        / "ui"
        / "fault_detector_ui.py"
    )
    source = path.read_text(encoding="utf-8")

    assert "from .sensor.controls import SensorControls" in source
    assert "SensorAttachmentClient" in source
    assert "self.sensor_controls = SensorControls(self)" in source
    assert "self.sensor_controls.select_requested.connect(" in source
    assert "self.sensor_controls.confirm_requested.connect(" not in source
    assert "self.sensor_confirm_button = QPushButton(\"✓\")" in source
    assert "QMessageBox.question(" in source
    assert (
        'self.tabs.addTab(self.sensor_controls, "Sensor Mounts")'
        in source
    )
