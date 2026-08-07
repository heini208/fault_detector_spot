"""Tests for explicit inspection object and routine commands."""

import py_trees
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import InspectionCommand

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands import (
    generic_complex_command,
)
from fault_detector_spot.behaviour_tree.nodes.inspection import (
    CreateInspectionDefinition,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    ReferenceTag,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


GenericCommand = generic_complex_command.GenericCommand


class FakeLogger:
    """Record behavior log messages."""

    def __init__(self):
        """Initialize log storage."""
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        """Record an informational message."""
        self.info_messages.append(message)

    def error(self, message):
        """Record an error message."""
        self.error_messages.append(message)


class FakeNode:
    """Provide the logger required by creation behaviors."""

    def __init__(self):
        """Initialize one logger."""
        self.logger = FakeLogger()

    def get_logger(self):
        """Return the recorded logger."""
        return self.logger


def create_writer(command):
    """Publish one internal command to the behavior blackboard."""
    writer = py_trees.blackboard.Client(name="CreationCommandWriter")
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = command
    return writer


def object_command(**values):
    """Create a valid object creation command."""
    fields = {
        "object_id": "motor_a",
        "display_name": "Motor A",
        "reference_tag_id": 23,
        "reference_tag_family": "36h11",
    }
    fields.update(values)
    inspection = InspectionCommand()
    for name, value in fields.items():
        setattr(inspection.object, name, value)
    return GenericCommand(
        CommandID.CREATE_INSPECTION_OBJECT,
        Time(sec=10),
        inspection=inspection,
    )


def routine_command(**values):
    """Create a valid routine creation command."""
    fields = {
        "object_id": "motor_a",
        "routine_id": "magnetic_scan",
        "display_name": "Magnetic scan",
        "sensor_id": "bmm150",
    }
    fields.update(values)
    inspection = InspectionCommand()
    inspection.object.object_id = fields.pop("object_id")
    for name, value in fields.items():
        setattr(inspection.routine, name, value)
    return GenericCommand(
        CommandID.CREATE_INSPECTION_ROUTINE,
        Time(sec=11),
        inspection=inspection,
    )


def run_behavior(tmp_path, command_id, command):
    """Set up and execute one creation behavior."""
    behavior = CreateInspectionDefinition(
        command_id,
        object_root=tmp_path,
    )
    node = FakeNode()
    behavior.setup(node=node)
    create_writer(command)
    return behavior.update(), behavior, node


def setup_function():
    """Clear shared blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear shared blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_create_object_command_persists_empty_object(tmp_path):
    """The object command creates only the explicitly requested object."""
    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_OBJECT,
        object_command(),
    )
    stored = ObjectRepository(tmp_path).load("motor_a")

    assert status == py_trees.common.Status.SUCCESS
    assert stored.display_name == "Motor A"
    assert stored.reference_tag.tag_id == 23
    assert stored.reference_tag.tag_family == "36h11"
    assert stored.routines == []
    assert behavior.feedback_message == "Created inspection object motor_a"
    assert len(node.logger.info_messages) == 1


def test_create_routine_command_persists_uncaptured_routine(tmp_path):
    """The routine command adds a null reference view to its object."""
    repository = ObjectRepository(tmp_path)
    repository.create(
        object_command_definition()
    )

    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_ROUTINE,
        routine_command(),
    )
    routine = repository.load("motor_a").get_routine(
        "magnetic_scan"
    )

    assert status == py_trees.common.Status.SUCCESS
    assert routine.display_name == "Magnetic scan"
    assert routine.sensor_id == "bmm150"
    assert routine.reference_view is None
    assert behavior.feedback_message == (
        "Created inspection routine motor_a/magnetic_scan"
    )
    assert len(node.logger.info_messages) == 1


def object_command_definition():
    """Return the domain object represented by object_command."""
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
    )


def test_duplicate_object_command_fails_without_overwrite(tmp_path):
    """Repeated object creation cannot replace existing data."""
    repository = ObjectRepository(tmp_path)
    repository.create(object_command_definition())

    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_OBJECT,
        object_command(display_name="Replacement"),
    )

    assert status == py_trees.common.Status.FAILURE
    assert repository.load("motor_a").display_name == "Motor A"
    assert "already exists" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_routine_command_requires_existing_object(tmp_path):
    """Routine creation never invents its missing parent object."""
    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_ROUTINE,
        routine_command(),
    )

    assert status == py_trees.common.Status.FAILURE
    assert ObjectRepository(tmp_path).list_object_ids() == []
    assert "does not exist" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_invalid_creation_payload_fails_before_persistence(tmp_path):
    """Missing required fields cannot create partial definitions."""
    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_OBJECT,
        object_command(reference_tag_family=""),
    )

    assert status == py_trees.common.Status.FAILURE
    assert ObjectRepository(tmp_path).list_object_ids() == []
    assert "Tag family must not be empty" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_missing_reference_tag_id_fails_before_persistence(tmp_path):
    """The message sentinel cannot become an accidental tag zero."""
    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_OBJECT,
        object_command(reference_tag_id=-1),
    )

    assert status == py_trees.common.Status.FAILURE
    assert ObjectRepository(tmp_path).list_object_ids() == []
    assert "reference_tag_id must be set" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_behavior_rejects_mismatched_command(tmp_path):
    """A selector branch cannot execute a different creation command."""
    status, behavior, node = run_behavior(
        tmp_path,
        CommandID.CREATE_INSPECTION_OBJECT,
        routine_command(),
    )

    assert status == py_trees.common.Status.FAILURE
    assert ObjectRepository(tmp_path).list_object_ids() == []
    assert "does not match" in behavior.feedback_message
    assert len(node.logger.error_messages) == 1
