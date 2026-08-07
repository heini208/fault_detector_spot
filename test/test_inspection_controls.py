"""Tests for inspection setup controls."""

import os
from dataclasses import replace

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.sensor_models import SensorDefinition

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

    def __init__(self, inspection_object_root):
        """Initialize UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = inspection_object_root
        self.sensor_definitions = [
            SensorDefinition(
                sensor_id="bmm150",
                display_name="BMM150 Hall sensor",
                hand_to_probe=PoseData.identity(),
            )
        ]

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
def controls(application, tmp_path):
    """Create isolated inspection controls."""
    ui = FakeUI(tmp_path)
    result = InspectionControls(ui)
    result.show_warning = lambda title, message: None
    result.ask_question = lambda title, message: True
    return result


def save_definition(controls, captured=False):
    """Persist one selectable object and routine for UI tests."""
    reference_view = None
    if captured:
        reference_view = ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_color_image_sensor",
            reference_dataset_path=(
                "reference_datasets/magnetic_scan/"
                "set_10_200000000/slot1_hand"
            ),
        )
    definition = InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id="bmm150",
                reference_views=(
                    [replace(
                        reference_view,
                        view_id="slot1_hand",
                        camera_id="hand",
                        slot_index=0,
                    )]
                    if reference_view is not None
                    else []
                ),
            )
        ],
    )
    ObjectRepository(
        controls.object_repository.root_dir
    ).create(definition)
    return definition


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
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("bmm150")
    )


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


def test_delete_object_publishes_selected_identifier(controls):
    """Object deletion targets a persisted dropdown selection."""
    save_definition(controls)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )

    assert controls.handle_delete_object() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.command.command_id == (
        CommandID.DELETE_INSPECTION_OBJECT
    )
    assert message.inspection.object.object_id == "motor_a"


def test_delete_routine_publishes_selected_identifiers(controls):
    """Routine deletion targets its persisted parent and routine."""
    save_definition(controls)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )

    assert controls.handle_delete_routine() is True

    message = controls.complex_command_publisher.messages[-1]
    assert message.command.command_id == (
        CommandID.DELETE_INSPECTION_ROUTINE
    )
    assert message.inspection.object.object_id == "motor_a"
    assert message.inspection.routine.routine_id == "magnetic_scan"


@pytest.mark.parametrize(
    "handler_name",
    ["handle_delete_object", "handle_delete_routine"],
)
def test_deletion_requires_saved_selection(controls, handler_name):
    """Typed identifiers alone cannot select a destructive target."""
    controls.object_id_field.setText("motor_a")
    controls.routine_id_field.setText("magnetic_scan")

    assert getattr(controls, handler_name)() is False
    assert controls.complex_command_publisher.messages == []


@pytest.mark.parametrize(
    "handler_name",
    ["handle_delete_object", "handle_delete_routine"],
)
def test_cancelled_deletion_does_not_publish(controls, handler_name):
    """Deletion confirmation cancellation preserves stored data."""
    save_definition(controls)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )
    controls.ask_question = lambda title, message: False

    assert getattr(controls, handler_name)() is False
    assert controls.complex_command_publisher.messages == []


def test_saved_object_selection_loads_definition_fields(controls):
    """Selecting a stored object fills the command-entry fields."""
    save_definition(controls)

    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )

    assert controls.object_id_field.text() == "motor_a"
    assert controls.object_display_name_field.text() == "Motor A"
    assert controls.reference_tag_id_field.text() == "23"
    assert controls.reference_tag_family_field.text() == "36h11"
    assert controls.saved_routine_dropdown.findData(
        "magnetic_scan"
    ) >= 0


def test_saved_routine_selection_loads_fields_and_capture_state(
    controls,
):
    """Routine selection exposes its configuration and capture state."""
    save_definition(controls, captured=True)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )

    assert controls.routine_id_field.text() == "magnetic_scan"
    assert controls.routine_display_name_field.text() == (
        "Magnetic scan"
    )
    assert controls.sensor_id_field.currentData() == "bmm150"
    assert controls.probe_frame_value_label.text() == "bmm150_probe"
    assert "Reference view: captured" in (
        controls.reference_view_status_label.text()
    )
    assert "10_200000000" in (
        controls.reference_view_status_label.text()
    )


def test_uncaptured_routine_is_identified(controls):
    """The UI distinguishes a configured routine from a taught one."""
    save_definition(controls)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )

    assert controls.reference_view_status_label.text() == (
        "Reference view: not captured"
    )


def test_storage_location_is_visible(controls):
    """The tab exposes the exact persistent object repository path."""
    assert str(controls.object_repository.root_dir) in (
        controls.storage_path_label.text()
    )


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


def test_routine_creation_requires_registered_sensor(controls):
    set_routine_fields(controls)
    controls.sensor_id_field.setCurrentIndex(0)

    assert controls.handle_create_routine() is False
    assert controls.complex_command_publisher.messages == []
