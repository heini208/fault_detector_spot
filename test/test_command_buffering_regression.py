"""Regression tests for semantic and internal command buffering."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import CommandStatus

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
)


class FakePublisher:
    def __init__(self, subscription_count=1):
        self.messages = []
        self.subscription_count = subscription_count

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscription_count


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.timer_callback = None
        self.logger = FakeLogger()

    def create_publisher(self, _message_type, topic, _qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_timer(self, _period, callback):
        self.timer_callback = callback
        return SimpleNamespace()

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=20, nanosec=5)
            )
        )

    def get_logger(self):
        return self.logger


def _request(command_id):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="test-ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )


def _status(request_id, state, buffered=0, detail=""):
    message = CommandStatus()
    message.request_id = request_id
    message.state = state
    message.buffered_command_count = buffered
    message.detail = detail
    return message


def test_semantic_requests_remain_fifo_buffered():
    node = FakeNode()
    controller = CommandController(node)
    dispatch = node.publishers[
        "fault_detector/_internal/commands/request"
    ]
    first = _request(CommandID.STAND_UP)
    second = _request(CommandID.READY_ARM)
    third = _request(CommandID.STOW_ARM)

    controller.submit(first)
    controller.submit(second)
    controller.submit(third)

    assert [item.request_id for item in dispatch.messages] == [
        first.request_id
    ]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (
        second.request_id,
        third.request_id,
    )

    controller.handle_command_status(
        _status(
            first.request_id,
            CommandStatus.STATE_RUNNING,
            buffered=2,
            detail="BT buffered composite request",
        )
    )
    controller.handle_command_status(
        _status(
            first.request_id,
            CommandStatus.STATE_SUCCEEDED,
            buffered=1,
        )
    )

    assert [item.request_id for item in dispatch.messages] == [
        first.request_id
    ]
    assert controller.active_request_id == first.request_id

    controller.handle_command_status(
        _status(
            first.request_id,
            CommandStatus.STATE_SUCCEEDED,
            buffered=0,
        )
    )

    assert [item.request_id for item in dispatch.messages] == [
        first.request_id,
        second.request_id,
    ]
    assert controller.active_request_id == second.request_id
    assert controller.queued_request_ids == (third.request_id,)

    controller.handle_command_status(
        _status(second.request_id, CommandStatus.STATE_SUCCEEDED)
    )

    assert [item.request_id for item in dispatch.messages] == [
        first.request_id,
        second.request_id,
        third.request_id,
    ]


def test_dispatched_request_reports_transport_stage():
    node = FakeNode()
    controller = CommandController(node)
    request = _request(CommandID.STAND_UP)
    states = []
    controller.add_status_listener(states.append)

    controller.submit(request)

    assert states[-1].detail.startswith(
        "Published stand_up to behavior tree"
    )
