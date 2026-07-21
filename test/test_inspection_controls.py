"""Tests for inspection setup controls."""

import os

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class FakePublisher:
    """Record published complex commands."""

    def __init__(self):
        """Initialize command storage."""
        self.messages = []

    def publish(self, message):
        """Record one message."""
        self.messages.append(message)


class FakeUI:
    """Provide the main UI contract used by inspection controls."""

    def __init__(self):
        """Initialize UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()

    def build_basic_command(self, command_id):
        """Build the command header used by controls."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by widgets."""
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    """Create isolated inspection controls."""
    ui = FakeUI()
    result = InspectionControls(ui)
    result.show_warning = lambda title, message: None
    result.ask_question = lambda title, message: True
    return result


def set_object_fields(controls):
    """Populate valid inspection-object fields."""
    controls.object_id_field.setText("motor_a")
    controls.object_display_name_field.setText("Motor A")
    controls.reference_tag_id_field.setText("23")
    controls.reference_tag_family_field.setText("36h11")


def set_routine_fields(controls):
    """Populate valid inspection-routine fields."""
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")
    controls.routine_display_name_field.setText("Magnetic scan")
    controls.sensor_id_field.setText("bmm150")
    controls.probe_frame_field.setText("sensor_tip")


def test_create_object_publishes_nested_definition(controls):
    """Object controls publish the complete nested object definition."""
    set_object_fields(controls)

    assert controls.handle_create_object() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.command.command_id == CommandID.CREATE_INSPECTION_OBJECT
    assert message.inspection.object.object_id == "motor_a"
    assert message.inspection.object.display_name == "Motor A"
    assert message.inspection.object.reference_tag_id == 23
    assert message.inspection.object.reference_tag_family == "36h11"


def test_create_routine_publishes_nested_definition(controls):
    """Routine controls publish the parent ID and routine definition."""
    set_routine_fields(controls)

    assert controls.handle_create_routine() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.command.command_id == CommandID.CREATE_INSPECTION_ROUTINE
    assert message.inspection.object.object_id == "motor_a"
    assert message.inspection.routine.routine_id == "magnetic_scan"
    assert message.inspection.routine.display_name == "Magnetic scan"
    assert message.inspection.routine.sensor_id == "bmm150"
    assert message.inspection.routine.probe_frame == "sensor_tip"


def test_capture_publishes_identifiers_without_replacement(controls):
    """Initial capture publishes identifiers and preserves overwrite safety."""
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")

    assert controls.handle_capture_reference_view() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.command.command_id == (
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
    )
    assert message.inspection.object.object_id == "motor_a"
    assert message.inspection.routine.routine_id == "magnetic_scan"
    assert message.inspection.replace_existing is False


def test_capture_requires_confirmation_before_replacement(controls):
    """A rejected replacement confirmation prevents publication."""
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")
    controls.replace_reference_view_checkbox.setChecked(True)
    controls.ask_question = lambda title, message: False

    assert controls.handle_capture_reference_view() is False
    assert controls.complex_command_publisher.messages == []


def test_confirmed_capture_enables_replacement(controls):
    """A confirmed replacement sets replace_existing on the command."""
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")
    controls.replace_reference_view_checkbox.setChecked(True)

    assert controls.handle_capture_reference_view() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.inspection.replace_existing is True


@pytest.mark.parametrize(
    "field_name",
    [
        "object_id_field",
        "object_display_name_field",
        "reference_tag_id_field",
        "reference_tag_family_field",
    ],
)
def test_object_creation_rejects_missing_required_input(
    controls,
    field_name,
):
    """Incomplete object definitions are not published."""
    set_object_fields(controls)
    getattr(controls, field_name).clear()

    assert controls.handle_create_object() is False
    assert controls.complex_command_publisher.messages == []


def test_object_creation_rejects_negative_tag_id(controls):
    """The UI does not publish the reserved missing-tag value."""
    set_object_fields(controls)
    controls.reference_tag_id_field.setText("-1")

    assert controls.handle_create_object() is False
    assert controls.complex_command_publisher.messages == []


@pytest.mark.parametrize(
    "field_name",
    [
        "object_id_field",
        "routine_id_field",
        "routine_display_name_field",
        "sensor_id_field",
        "probe_frame_field",
    ],
)
def test_routine_creation_rejects_missing_required_input(
    controls,
    field_name,
):
    """Incomplete routine definitions are not published."""
    set_routine_fields(controls)
    getattr(controls, field_name).clear()

    assert controls.handle_create_routine() is False
    assert controls.complex_command_publisher.messages == []
