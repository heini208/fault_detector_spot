"""Tests for command dispatch readiness."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time

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
    def __init__(self):
        self.messages = []
        self.subscription_count = 0

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscription_count


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.timer_callback = None

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
                to_msg=lambda: Time(sec=9, nanosec=2)
            )
        )

    def get_logger(self):
        return SimpleNamespace(
            info=lambda _message: None,
            error=lambda _message: None,
        )


def request(command_id):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="navigation-ui",
        origin=CommandOrigin.NAVIGATION_SETUP,
        recording_policy=RecordingPolicy.EXCLUDE,
    )


def test_request_waits_until_bt_consumer_exists():
    node = FakeNode()
    controller = CommandController(node)
    pending = request(CommandID.SWAP_MAP)

    controller.submit(pending)

    assert controller.active_request_id == ""
    assert controller.queued_request_ids == (pending.request_id,)

    node.publishers[
        "fault_detector/_internal/commands/request"
    ].subscription_count = 1
    node.timer_callback()

    assert controller.active_request_id == pending.request_id


def test_emergency_is_not_lost_when_consumer_is_absent():
    node = FakeNode()
    controller = CommandController(node)
    controller.submit(request(CommandID.SWAP_MAP))

    emergency_id = controller.cancel_all("operator-ui")

    assert controller.active_request_id == ""
    assert controller.queued_request_ids == (emergency_id,)

    node.publishers[
        "fault_detector/_internal/commands/request"
    ].subscription_count = 1
    node.timer_callback()

    assert controller.active_request_id == emergency_id
