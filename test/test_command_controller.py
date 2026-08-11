"""Tests for serialized semantic command execution."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import CommandStatus, ComplexCommand

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    DuplicateCommandRequest,
)


class FakePublisher:
    """Collect messages published on one topic."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    """Collect controller errors."""

    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class FakeNode:
    """Provide the ROS node surface used by the controller."""

    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.logger = FakeLogger()

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=17, nanosec=23)
            )
        )

    def get_logger(self):
        return self.logger


def make_request(command_id, client_id="operator_ui"):
    """Build one valid operational request."""
    command = ComplexCommand()
    command.command.command_id = command_id
    return CommandRequest.create(
        command=command,
        client_id=client_id,
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )


def command_status(request_id, state, buffered_count=0):
    """Build one behavior-tree result."""
    message = CommandStatus()
    message.request_id = request_id
    message.state = state
    message.buffered_command_count = buffered_count
    return message


def test_controller_dispatches_only_one_semantic_request_at_a_time():
    node = FakeNode()
    controller = CommandController(node)
    states = []
    controller.add_status_listener(states.append)
    first = make_request(CommandID.STAND_UP.value)
    second = make_request(CommandID.STOW_ARM.value)

    controller.submit(first)
    controller.submit(second)

    dispatched = node.publishers[
        "fault_detector/commands/request"
    ].messages
    accepted = node.publishers[
        "fault_detector/commands/accepted"
    ].messages
    assert len(accepted) == 2
    assert [message.request_id for message in dispatched] == [
        first.request_id
    ]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (second.request_id,)

    controller.handle_command_status(command_status(
        first.request_id,
        CommandStatus.STATE_SUCCEEDED,
    ))

    assert [message.request_id for message in dispatched] == [
        first.request_id,
        second.request_id,
    ]
    assert controller.active_request_id == second.request_id
    assert states[-2].state is CommandControllerState.SUCCEEDED
    assert states[-1].state is CommandControllerState.DISPATCHED


def test_internal_success_does_not_release_the_next_request():
    node = FakeNode()
    controller = CommandController(node)
    first = make_request(CommandID.MOVE_ARM_TO_TAG_AND_WAIT.value)
    second = make_request(CommandID.STOW_ARM.value)
    controller.submit(first)
    controller.submit(second)

    handled = controller.handle_command_status(command_status(
        first.request_id,
        CommandStatus.STATE_SUCCEEDED,
        buffered_count=1,
    ))

    assert handled is True
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (second.request_id,)
    assert len(node.publishers[
        "fault_detector/commands/request"
    ].messages) == 1


def test_duplicate_request_identity_is_rejected():
    controller = CommandController(FakeNode())
    request = make_request(CommandID.STAND_UP.value)
    controller.submit(request)

    with pytest.raises(DuplicateCommandRequest):
        controller.submit(request)


def test_emergency_stop_clears_queue_and_bypasses_active_request():
    node = FakeNode()
    controller = CommandController(node)
    states = []
    controller.add_status_listener(states.append)
    first = make_request(CommandID.MOVE_TO_WAYPOINT.value)
    second = make_request(CommandID.STOW_ARM.value)
    controller.submit(first)
    controller.submit(second)

    emergency_request_id = controller.cancel_all()

    dispatched = node.publishers[
        "fault_detector/commands/request"
    ].messages
    assert [message.request_id for message in dispatched] == [
        first.request_id,
        emergency_request_id,
    ]
    assert controller.active_request_id == emergency_request_id
    assert controller.queued_request_ids == ()
    cancelled = {
        status.request_id
        for status in states
        if status.state is CommandControllerState.CANCELLED
    }
    assert cancelled == {first.request_id, second.request_id}
    assert (
        dispatched[-1].command.command.command_id
        == CommandID.EMERGENCY_CANCEL.value
    )


def test_dispatch_rewrites_nested_identity_and_timestamp():
    node = FakeNode()
    controller = CommandController(node)
    request = make_request(CommandID.READY_ARM.value)

    controller.submit(request)

    message = node.publishers[
        "fault_detector/commands/request"
    ].messages[0]
    assert message.command.command.request_id == request.request_id
    assert message.command.command.header.stamp == Time(
        sec=17,
        nanosec=23,
    )
