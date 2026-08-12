"""Tests for machine-readable command request correlation."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.request_identity import new_request_id
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=7, nanosec=11))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def semantic_request(command_id, client_id, context_id="", origin=None):
    if origin is None:
        origin = CommandOrigin.OPERATIONAL
    recording_policy = (
        RecordingPolicy.EXCLUDE
        if origin is CommandOrigin.PROBE_SETUP
        else RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
    )
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id=client_id,
        context_id=context_id,
        origin=origin,
        recording_policy=recording_policy,
    )


def test_semantic_command_request_id_reaches_internal_motion():
    request = semantic_request(CommandID.STAND_UP, "operator_ui")
    subscriber = CommandSubscriber()
    subscriber.blackboard = SimpleNamespace(command_buffer=[])
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())

    subscriber.fire_request(request)

    assert len(subscriber.blackboard.command_buffer) == 1
    assert subscriber.blackboard.command_buffer[0].request_id == request.request_id


def test_request_metadata_reaches_internal_motion():
    request = semantic_request(
        CommandID.MOVE_ARM_RELATIVE,
        "probe_ui",
        context_id="probe_setup_8",
        origin=CommandOrigin.PROBE_SETUP,
    )
    subscriber = CommandSubscriber()
    subscriber.blackboard = SimpleNamespace(command_buffer=[])
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())

    subscriber.fire_request(request)

    command = subscriber.blackboard.command_buffer[0]
    assert command.request_id == request.request_id
    assert command.client_id == "probe_ui"
    assert command.context_id == "probe_setup_8"
    assert command.origin is CommandOrigin.PROBE_SETUP
    assert command.recording_policy is RecordingPolicy.EXCLUDE


def test_empty_request_translation_reports_correlated_failure():
    request = semantic_request(CommandID.SCAN_ALL_IN_RANGE, "operator_ui")
    subscriber = CommandSubscriber()
    subscriber.blackboard = SimpleNamespace(
        command_buffer=[],
        reachable_tags={},
    )
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())
    subscriber.request_status_publisher = FakePublisher()
    subscriber.pending_msgs = [(Time(), request)]

    subscriber.update()

    result = subscriber.request_status_publisher.messages[0]
    assert result.request_id == request.request_id
    assert result.state == CommandStatus.STATE_FAILED
    assert "no executable" in result.detail


def test_structured_status_preserves_request_and_buffer_count():
    request_id = new_request_id()
    command = SimpleNamespace(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        request_id=request_id,
    )
    publisher = BufferStatusPublisher()
    publisher.node = SimpleNamespace(get_clock=lambda: FakeClock())
    publisher.blackboard = SimpleNamespace(
        last_command=command,
        command_tree_status=Status.RUNNING,
    )

    message = publisher.get_structured_status_message(2)

    assert message.request_id == request_id
    assert message.command_id == CommandID.MOVE_ARM_TO_TAG
    assert message.state == CommandStatus.STATE_RUNNING
    assert message.buffered_command_count == 2


def test_terminal_status_is_not_replaced_by_idle_guard_failure():
    request_id = new_request_id()
    command = SimpleNamespace(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        request_id=request_id,
    )
    publisher = BufferStatusPublisher()
    publisher.node = SimpleNamespace(get_clock=lambda: FakeClock())
    publisher.blackboard = SimpleNamespace(
        command_buffer=[],
        last_command=command,
        command_tree_status=Status.SUCCESS,
    )
    publisher.buffer_pub = FakePublisher()
    publisher.structured_status_pub = FakePublisher()

    publisher.update()
    publisher.blackboard.command_tree_status = Status.FAILURE
    publisher.update()

    messages = publisher.structured_status_pub.messages
    assert len(messages) == 1
    assert messages[0].request_id == request_id
    assert messages[0].state == CommandStatus.STATE_SUCCEEDED
