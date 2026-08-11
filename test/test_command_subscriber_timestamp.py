"""Tests for command handling when the ROS clock does not advance."""

from fault_detector_msgs.msg import ComplexCommand

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)


class FakeBlackboard:
    """Provide the command buffer used by the subscriber."""

    def __init__(self):
        """Create an empty command buffer."""
        self.command_buffer = []


def make_request(command_id):
    """Create one request at a fixed simulated timestamp."""
    command = ComplexCommand()
    command.command.command_id = command_id
    command.command.header.stamp.sec = 42
    command.command.header.stamp.nanosec = 0
    return CommandRequest.create(
        command=command,
        client_id="test_client",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )


def test_same_simulated_timestamp_does_not_drop_distinct_commands():
    """A static simulation clock cannot suppress a later UI action."""
    subscriber = CommandSubscriber()
    subscriber.blackboard = FakeBlackboard()

    subscriber.fire_request(
        make_request(CommandID.CREATE_INSPECTION_ROUTINE)
    )
    subscriber.fire_request(
        make_request(
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
