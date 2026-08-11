"""Tests for command handling when the ROS clock does not advance."""

from fault_detector_msgs.msg import BasicCommand

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)


class FakeBlackboard:
    """Provide the command buffer used by the subscriber."""

    def __init__(self):
        """Create an empty command buffer."""
        self.command_buffer = []


def make_command(command_id):
    """Create one command at a fixed simulated timestamp."""
    command = BasicCommand()
    command.command_id = command_id
    command.header.stamp.sec = 42
    command.header.stamp.nanosec = 0
    return command


def test_same_simulated_timestamp_does_not_drop_distinct_commands():
    """A static simulation clock cannot suppress a later UI action."""
    subscriber = CommandSubscriber()
    subscriber.blackboard = FakeBlackboard()

    subscriber.fire_command(
        make_command(CommandID.CREATE_INSPECTION_ROUTINE)
    )
    subscriber.fire_command(
        make_command(
            CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        )
    )

    assert [
        command.command_id
        for command in subscriber.blackboard.command_buffer
    ] == [
        CommandID.CREATE_INSPECTION_ROUTINE,
        CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW,
    ]
