"""Tests for command handling when the ROS clock does not advance."""

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_to_message,
)


class FakeLogger:
    def info(self, _message):
        pass


class FakeNode:
    def get_logger(self):
        return FakeLogger()


def wire_request(command_id):
    request = CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="test_client",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )
    message = command_request_to_message(request)
    message.header.stamp.sec = 42
    return message


def test_same_transport_timestamp_keeps_distinct_semantic_requests():
    subscriber = CommandSubscriber()
    subscriber.node = FakeNode()

    subscriber.append_request_to_buffer(wire_request(CommandID.STAND_UP))
    subscriber.append_request_to_buffer(wire_request(CommandID.STOW_ARM))

    assert len(subscriber.pending_msgs) == 2
    assert all(
        isinstance(request.command, SemanticCommand)
        for _, request in subscriber.pending_msgs
    )
    assert [
        request.command.command_id
        for _, request in subscriber.pending_msgs
    ] == [
        CommandID.STAND_UP,
        CommandID.STOW_ARM,
    ]
