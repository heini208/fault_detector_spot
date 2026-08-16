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
    assert not hasattr(controls, "new_mount_button")
    assert controls.retire_mount_button.isEnabled() is False
    assert controls.save_mount_button.isEnabled() is True
    assert controls.discard_mount_button.isEnabled() is True
    assert controls.mount_id_field.isReadOnly() is False
    assert controls.display_name_field.isReadOnly() is False
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
    assert not hasattr(controls, "new_mount_button")


def test_sensor_definition_form_is_immediately_editable(application):
    controls = SensorControls()
    controls.mount_id_field.setText("bmm150_mount")

    assert controls.mount_id_field.isReadOnly() is False
    assert controls.display_name_field.isReadOnly() is False
    assert controls.probe_frame_field.isReadOnly() is True
    assert controls.probe_frame_field.text() == "bmm150_mount_probe"
    assert [field.text() for field in controls.translation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]
    assert [field.text() for field in controls.rotation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]
    assert controls.save_mount_button.isEnabled() is True
    assert controls.discard_mount_button.isEnabled() is True


def test_manual_sensor_form_emits_creation_intent(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.mount_id_field.setText("bmm150_mount")
    controls.display_name_field.setText("BMM150 Hall sensor")
    values = ("0.20", "-0.01", "0.03")
    for field, value in zip(controls.translation_fields, values):
        field.setText(value)
    rotations = ("0.0", "0.0", "90.0")
    for field, value in zip(controls.rotation_fields, rotations):
        field.setText(value)
    intents = []
    controls.create_requested.connect(intents.append)

    controls.save_mount_button.click()

    assert len(intents) == 1
    intent = intents[0]
    assert intent.sensor_id == "bmm150_mount"
    assert intent.display_name == "BMM150 Hall sensor"
    assert intent.translation_m == pytest.approx((0.20, -0.01, 0.03))
    assert intent.rotation_degrees == pytest.approx((0.0, 0.0, 90.0))



def test_clear_fields_resets_definition_without_new_action(application):
    controls = SensorControls()
    controls.mount_id_field.setText("draft_sensor")
    controls.display_name_field.setText("Draft sensor")
    controls.translation_fields[0].setText("0.25")

    controls.discard_mount_button.click()

    assert controls.mount_id_field.text() == ""
    assert controls.display_name_field.text() == ""
    assert controls.probe_frame_field.text() == ""
    assert [field.text() for field in controls.translation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]
    assert controls.save_mount_button.isEnabled() is True



def test_sensor_creation_result_keeps_failed_draft_editable(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.mount_id_field.setText("bad sensor")
    controls.display_name_field.setText("Bad sensor")

    controls.mark_sensor_creation_pending()
    controls.finish_sensor_creation(False, "Sensor ID is invalid")

    assert controls.mount_id_field.text() == "bad sensor"
    assert controls.mount_id_field.isReadOnly() is False
    assert controls.save_mount_button.isEnabled() is True
    assert controls.discard_mount_button.isEnabled() is True
    assert controls.calibration_status_value.text() == "Sensor ID is invalid"


def test_successful_sensor_creation_locks_saved_definition(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.mount_id_field.setText("hall_probe")
    controls.display_name_field.setText("Hall probe")

    controls.mark_sensor_creation_pending()
    controls.finish_sensor_creation(True, "Created sensor 'hall_probe'")

    assert controls.mount_id_field.text() == ""
    assert controls.display_name_field.text() == ""
    assert controls.mount_id_field.isReadOnly() is False
    assert controls.display_name_field.isReadOnly() is False
    assert controls.save_mount_button.isEnabled() is True
    assert controls.discard_mount_button.isEnabled() is True
    assert [field.text() for field in controls.translation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]
    assert [field.text() for field in controls.rotation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]
    assert controls.calibration_status_value.text() == (
        "Created sensor 'hall_probe'"
    )


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
    assert "self.sensor_controls.create_requested.connect(" in source
    assert "self.sensor_controls.confirm_requested.connect(" not in source
    assert "self.sensor_attachment_client.create_sensor(" in source
    assert "self.sensor_confirm_button = QPushButton(\"✓\")" in source
    assert "QMessageBox.question(" in source
    assert (
        'self.tabs.addTab(self.sensor_controls, "Sensor Mounts")'
        in source
    )
