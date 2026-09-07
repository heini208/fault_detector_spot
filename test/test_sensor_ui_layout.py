"""Tests for the presentation-only sensor mount UI."""

import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.sensor.controls import SensorControls
from fault_detector_spot.ui.sensor.models import SensorTopicSuggestion


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def definition(
    sensor_id="test",
    display_name="Test sensor",
    position=(0.20, -0.01, 0.03),
    rotation_degrees=(0.0, 0.0, 90.0),
    channels=(),
):
    return SimpleNamespace(
        sensor_id=sensor_id,
        display_name=display_name,
        probe_frame=f"{sensor_id}_probe",
        position=position,
        orientation=(0.0, 0.0, 0.0, 1.0),
        rotation_degrees=rotation_degrees,
        channels=channels,
    )


def channel(
    channel_id="magnetic_field",
    topic="/sensors/bmm150_probe/magnetic_field",
    message_type="sensor_msgs/msg/MagneticField",
):
    return SimpleNamespace(
        channel_id=channel_id,
        topic=topic,
        message_type=message_type,
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


def test_sensor_definition_form_is_immediately_editable(application):
    controls = SensorControls()
    controls.mount_id_field.setText("test")

    assert controls.mount_id_field.isReadOnly() is False
    assert controls.display_name_field.isReadOnly() is False
    assert controls.probe_frame_field.isReadOnly() is True
    assert controls.probe_frame_field.text() == "test_probe"
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


def test_acquisition_channels_are_collapsed_by_default(application):
    controls = SensorControls()

    assert controls.channel_content.isHidden()
    controls.channel_toggle_button.click()
    assert not controls.channel_content.isHidden()


def test_current_attachment_does_not_expose_internal_revision(application):
    controls = SensorControls()

    assert not hasattr(controls, "attachment_revision_value")


def test_registry_labels_manual_transform_without_claiming_calibration(
    application,
):
    controls = SensorControls()
    controls.apply_definitions((definition(),))

    assert controls.mount_table.horizontalHeaderItem(3).text() == (
        "Transform source"
    )
    assert controls.mount_table.item(0, 3).text() == "Manual"
    assert "Calibrated" not in controls.mount_table.item(0, 3).text()


def test_create_form_accepts_test_as_mount_id(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.mount_id_field.setText("test")
    controls.display_name_field.setText("Test sensor")
    intents = []
    controls.create_requested.connect(intents.append)

    controls.save_mount_button.click()

    assert len(intents) == 1
    assert intents[0].sensor_id == "test"
    assert intents[0].display_name == "Test sensor"
    assert intents[0].channels == ()


def test_mount_form_adds_channel_to_definition_intent(application):
    controls = SensorControls()
    controls.mount_id_field.setText("bmm150_probe")
    controls.display_name_field.setText("BMM150 probe")
    controls.channel_id_field.setText("magnetic_field")
    controls.channel_topic_field.setEditText(
        "/sensors/bmm150_probe/magnetic_field"
    )
    controls.channel_message_type_field.setEditText(
        "sensor_msgs/msg/MagneticField"
    )
    controls.save_channel_button.click()
    intents = []
    controls.create_requested.connect(intents.append)

    controls.save_mount_button.click()

    assert controls.channel_table.rowCount() == 1
    assert len(intents) == 1
    assert intents[0].channels[0].channel_id == "magnetic_field"
    assert intents[0].channels[0].topic == (
        "/sensors/bmm150_probe/magnetic_field"
    )


def test_live_topic_suggestion_fills_advertised_message_type(application):
    controls = SensorControls()
    controls.apply_topic_suggestions(
        (
            SensorTopicSuggestion(
                topic="/sensors/bmm150_probe/magnetic_field",
                message_types=("sensor_msgs/msg/MagneticField",),
            ),
            SensorTopicSuggestion(
                topic="/active_map",
                message_types=("nav_msgs/msg/OccupancyGrid",),
            ),
        )
    )

    controls.channel_topic_field.setCurrentText(
        "/sensors/bmm150_probe/magnetic_field"
    )

    assert controls.channel_topic_field.findText(
        "/sensors/bmm150_probe/magnetic_field"
    ) >= 0
    assert controls.channel_topic_field.itemText(0) == (
        "/sensors/bmm150_probe/magnetic_field"
    )
    assert controls.channel_message_type_field.currentText() == (
        "sensor_msgs/msg/MagneticField"
    )


def test_live_suggestions_preserve_manually_entered_offline_channel(
    application,
):
    controls = SensorControls()
    controls.channel_topic_field.setEditText("/offline/custom/value")
    controls.channel_message_type_field.setEditText(
        "custom_msgs/msg/Value"
    )

    controls.apply_topic_suggestions(
        (
            SensorTopicSuggestion(
                topic="/online/value",
                message_types=("std_msgs/msg/Float64",),
            ),
        )
    )

    assert controls.channel_topic_field.currentText() == (
        "/offline/custom/value"
    )
    assert controls.channel_message_type_field.currentText() == (
        "custom_msgs/msg/Value"
    )


def test_edit_mount_populates_and_updates_channel_rows(application):
    controls = SensorControls()
    controls.apply_definitions((definition(channels=(channel(),)),))
    controls.apply_attachment_state(attachment("active"))
    controls.mount_table.selectRow(0)
    controls.edit_mount_button.click()

    assert controls.channel_table.rowCount() == 1
    assert controls.channel_table.item(0, 0).text() == "magnetic_field"

    controls.channel_table.selectRow(0)
    controls.channel_id_field.setText("field")
    controls.save_channel_button.click()
    updates = []
    controls.update_requested.connect(updates.append)
    controls.save_mount_button.click()

    assert len(updates) == 1
    assert updates[0].channels[0].channel_id == "field"


def test_detected_head_id_is_optional_autofill(application):
    controls = SensorControls()
    connection = SimpleNamespace(
        status=SimpleNamespace(value="unassigned"),
        expected_sensor_id="",
        connected_sensor_ids=("bmm150_probe",),
        detail="Connected sensor head is not assigned",
    )

    controls.apply_sensor_head_connection(connection)

    assert controls.mount_id_field.text() == ""
    assert controls.detected_mount_dropdown.currentData() == "bmm150_probe"
    assert controls.use_detected_mount_button.isEnabled()

    controls.use_detected_mount_button.click()

    assert controls.mount_id_field.text() == "bmm150_probe"
    controls.mount_id_field.setText("manual_offline_probe")
    assert controls.mount_id_field.text() == "manual_offline_probe"


def test_no_connected_head_does_not_disable_manual_creation(application):
    controls = SensorControls()
    connection = SimpleNamespace(
        status=SimpleNamespace(value="no_heads"),
        expected_sensor_id="",
        connected_sensor_ids=(),
        detail="No sensor heads are connected",
    )

    controls.apply_sensor_head_connection(connection)

    assert not controls.use_detected_mount_button.isEnabled()
    assert controls.mount_id_field.isEnabled()
    assert controls.save_mount_button.isEnabled()


def test_existing_id_requires_explicit_edit_mode(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.mount_id_field.setText("test")
    controls.display_name_field.setText("Other name")
    intents = []
    controls.create_requested.connect(intents.append)

    controls.save_mount_button.click()

    assert intents == []
    assert "already exists" in controls.transform_status_value.text()
    assert "Edit Sensor" in controls.transform_status_value.text()


def test_edit_selected_sensor_populates_existing_transform(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.apply_attachment_state(attachment("active"))
    controls.mount_table.selectRow(0)

    assert controls.edit_mount_button.isEnabled() is True
    controls.edit_mount_button.click()

    assert controls.mount_id_field.text() == "test"
    assert controls.mount_id_field.isReadOnly() is True
    assert controls.display_name_field.text() == "Test sensor"
    assert [field.text() for field in controls.translation_fields] == [
        "0.2",
        "-0.01",
        "0.03",
    ]
    assert [field.text() for field in controls.rotation_fields] == [
        "0",
        "0",
        "90",
    ]


def test_edit_save_emits_update_not_create(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.apply_attachment_state(attachment("active"))
    controls.mount_table.selectRow(0)
    controls.edit_mount_button.click()
    controls.translation_fields[0].setText("0.25")
    creates = []
    updates = []
    controls.create_requested.connect(creates.append)
    controls.update_requested.connect(updates.append)

    controls.save_mount_button.click()

    assert creates == []
    assert len(updates) == 1
    assert updates[0].sensor_id == "test"
    assert updates[0].translation_m[0] == pytest.approx(0.25)


def test_edit_and_delete_are_disabled_for_selected_attachment(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.apply_attachment_state(
        attachment("active", active_sensor_id="test")
    )
    controls.mount_table.selectRow(0)

    assert controls.edit_mount_button.isEnabled() is False
    assert controls.delete_mount_button.isEnabled() is False
    assert "Remove or switch" in controls.edit_mount_button.toolTip()


def test_delete_selected_emits_exact_sensor_id(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.apply_attachment_state(attachment("active"))
    controls.mount_table.selectRow(0)
    deleted = []
    controls.delete_requested.connect(deleted.append)

    controls.delete_mount_button.click()

    assert deleted == ["test"]


def test_discard_changes_returns_to_new_sensor_form(application):
    controls = SensorControls()
    controls.apply_definitions((definition(),))
    controls.apply_attachment_state(attachment("active"))
    controls.mount_table.selectRow(0)
    controls.edit_mount_button.click()

    controls.discard_mount_button.click()

    assert controls.mount_id_field.text() == ""
    assert controls.mount_id_field.isReadOnly() is False
    assert controls.display_name_field.text() == ""
    assert [field.text() for field in controls.translation_fields] == [
        "0.0",
        "0.0",
        "0.0",
    ]


def test_no_sensor_state_uses_real_hand_frame(application):
    controls = SensorControls()
    controls.apply_definitions(())
    controls.apply_attachment_state(
        attachment("none", attachment_revision=4)
    )

    assert controls.mount_table.rowCount() == 0
    assert controls.attachment_name_value.text() == "No sensor"
    assert controls.attachment_status_value.text() == "Confirmation pending"
    assert controls.attachment_probe_frame_value.text() == "hand"


def test_sensor_workspace_has_no_sensing_elements_placeholder(application):
    controls = SensorControls()

    assert not hasattr(controls, "manage_sensing_elements_button")


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


def test_main_ui_wires_create_update_and_delete_sensor_intents():
    path = (
        Path(__file__).parents[1]
        / "fault_detector_spot"
        / "ui"
        / "fault_detector_ui.py"
    )
    source = path.read_text(encoding="utf-8")

    assert "self.sensor_controls.create_requested.connect(" in source
    assert "self.sensor_controls.update_requested.connect(" in source
    assert "self.sensor_controls.delete_requested.connect(" in source
    assert "self.sensor_registry_client.create_sensor" in source
    assert "self.sensor_registry_client.update_sensor" in source
    assert "self.sensor_registry_client.delete_sensor(" in source
    assert "self.sensor_attachment_client.create_sensor" not in source
    assert "self.sensor_attachment_client.update_sensor" not in source
    assert "self.sensor_attachment_client.delete_sensor(" not in source
    assert '"Delete sensor mount"' in source
