"""Tests for inspection-definition deletion commands."""

import py_trees
import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import InspectionCommand

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.behaviour_tree.nodes.inspection import (
    DeleteInspectionDefinition,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


class FakeLogger:
    """Record deletion behavior output."""

    def __init__(self):
        """Initialize log storage."""
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        """Record an info message."""
        self.info_messages.append(message)

    def error(self, message):
        """Record an error message."""
        self.error_messages.append(message)


class FakeNode:
    """Provide logging required by the deletion behavior."""

    def __init__(self):
        """Initialize the fake logger."""
        self.logger = FakeLogger()

    def get_logger(self):
        """Return the recorded logger."""
        return self.logger


def make_object():
    """Create one object with one routine."""
    return InspectionObject(
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
            )
        ],
    )


def make_command(command_id, object_id="motor_a", routine_id=""):
    """Create one internal deletion command."""
    inspection = InspectionCommand()
    inspection.object.object_id = object_id
    inspection.routine.routine_id = routine_id
    return GenericCommand(
        command_id=command_id,
        stamp=Time(sec=10),
        inspection=inspection,
    )


def write_command(command):
    """Publish one command to the behavior blackboard."""
    writer = py_trees.blackboard.Client(name="DeletionCommandWriter")
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = command
    return writer


def make_behavior(tmp_path, command_id):
    """Configure one deletion behavior with isolated storage."""
    behavior = DeleteInspectionDefinition(
        command_id=command_id,
        object_root=tmp_path,
    )
    node = FakeNode()
    behavior.setup(node=node)
    return behavior, node


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_delete_object_command_removes_complete_object(tmp_path):
    """Object deletion succeeds only after removing persisted data."""
    repository = ObjectRepository(tmp_path)
    repository.create(make_object())
    behavior, node = make_behavior(
        tmp_path,
        CommandID.DELETE_INSPECTION_OBJECT,
    )
    write_command(make_command(CommandID.DELETE_INSPECTION_OBJECT))

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert not repository.get_object_dir("motor_a").exists()
    assert node.logger.info_messages == [
        "Deleted inspection object motor_a"
    ]


def test_delete_routine_command_preserves_parent_object(tmp_path):
    """Routine deletion removes only the selected child definition."""
    repository = ObjectRepository(tmp_path)
    repository.create(make_object())
    behavior, node = make_behavior(
        tmp_path,
        CommandID.DELETE_INSPECTION_ROUTINE,
    )
    write_command(make_command(
        CommandID.DELETE_INSPECTION_ROUTINE,
        routine_id="magnetic_scan",
    ))

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert repository.load("motor_a").routines == []
    assert node.logger.info_messages == [
        "Deleted inspection routine motor_a/magnetic_scan"
    ]


@pytest.mark.parametrize(
    ("command_id", "object_id", "routine_id", "expected"),
    [
        (
            CommandID.DELETE_INSPECTION_OBJECT,
            "missing",
            "",
            "does not exist",
        ),
        (
            CommandID.DELETE_INSPECTION_ROUTINE,
            "motor_a",
            "missing",
            "does not exist",
        ),
        (
            CommandID.DELETE_INSPECTION_ROUTINE,
            "motor_a",
            "",
            "routine_id",
        ),
    ],
)
def test_invalid_deletion_target_fails_without_false_success(
    tmp_path,
    command_id,
    object_id,
    routine_id,
    expected,
):
    """Missing identifiers and definitions produce command failure."""
    ObjectRepository(tmp_path).create(make_object())
    behavior, node = make_behavior(tmp_path, command_id)
    write_command(make_command(command_id, object_id, routine_id))

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert expected in behavior.feedback_message
    assert len(node.logger.error_messages) == 1


def test_unsupported_deletion_behavior_is_rejected():
    """The deletion behavior cannot be assigned another command."""
    with pytest.raises(ValueError, match="Unsupported"):
        DeleteInspectionDefinition(CommandID.STOW_ARM)
