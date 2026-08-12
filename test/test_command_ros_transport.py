"""Tests for the ROS adapter around the pure command controller."""

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
    CommandControllerState,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_to_message,
)
from fault_detector_spot.application.ros.command_transport import (
    RosCommandTransport,
)


class FakePublisher:
    def __init__(self, subscription_count=0):
        self.messages = []
        self.subscription_count = subscription_count

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscription_count


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


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
        self.timers = []
        self.logger = FakeLogger()

    def create_publisher(self, _message_type, topic, _qos):
        count = 1 if topic == "fault_detector/_internal/commands/request" else 0
        publisher = FakePublisher(subscription_count=count)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_timer(self, _period, callback):
        timer = FakeTimer(callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=17, nanosec=23)
            )
        )

    def get_logger(self):
        return self.logger


def make_request(command_id):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="operator-ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )


def test_ros_submission_is_converted_before_entering_controller():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.READY_ARM)

    accepted = node.subscriptions[
        "fault_detector/_internal/commands/submit"
    ](command_request_to_message(request))

    assert accepted is True
    assert controller.active_request_id == request.request_id
    assert controller._active.command.command_id is CommandID.READY_ARM
    accepted_message = node.publishers[
        "fault_detector/_internal/commands/accepted"
    ].messages[0]
    assert accepted_message.request_id == request.request_id


def test_transport_adds_wire_identity_and_timestamp_on_dispatch():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.STAND_UP)

    controller.submit(request)

    message = node.publishers[
        "fault_detector/_internal/commands/request"
    ].messages[0]
    assert message.request_id == request.request_id
    assert message.command.command.request_id == request.request_id
    assert message.command.command.header.stamp == Time(
        sec=17,
        nanosec=23,
    )


def test_bt_status_is_translated_back_to_controller_feedback():
    node = FakeNode()
    controller = CommandController()
    states = []
    controller.add_status_listener(states.append)
    RosCommandTransport(node, controller)
    first = make_request(CommandID.STAND_UP)
    second = make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)

    running = CommandStatus()
    running.request_id = first.request_id
    running.state = CommandStatus.STATE_RUNNING
    running.buffered_command_count = 1
    node.subscriptions[
        "fault_detector/_internal/command_status"
    ](running)

    assert states[-1].state is CommandControllerState.RUNNING
    assert controller.active_request_id == first.request_id

    succeeded = CommandStatus()
    succeeded.request_id = first.request_id
    succeeded.state = CommandStatus.STATE_SUCCEEDED
    succeeded.buffered_command_count = 0
    node.subscriptions[
        "fault_detector/_internal/command_status"
    ](succeeded)

    assert controller.active_request_id == second.request_id


def test_invalid_ros_submission_is_rejected_before_queueing():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.STAND_UP)
    message = command_request_to_message(request)
    message.command.command.request_id = make_request(
        CommandID.STOW_ARM
    ).request_id

    accepted = node.subscriptions[
        "fault_detector/_internal/commands/submit"
    ](message)

    assert accepted is False
    assert controller.active_request_id == ""
    rejection = node.publishers[
        "fault_detector/_internal/commands/status"
    ].messages[0]
    assert rejection.request_id == request.request_id
    assert rejection.state == CommandStatus.STATE_FAILED


def test_transport_readiness_controls_pure_controller_dispatch():
    node = FakeNode()
    node_override = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node_override, controller)
    dispatch = node_override.publishers[
        "fault_detector/_internal/commands/request"
    ]
    dispatch.subscription_count = 0
    request = make_request(CommandID.READY_ARM)

    controller.submit(request)

    assert controller.active_request_id == ""
    assert controller.queued_request_ids == (request.request_id,)

    dispatch.subscription_count = 1
    node_override.timers[0].callback()

    assert controller.active_request_id == request.request_id
    assert len(dispatch.messages) == 1
    transport.close()
    assert node_override.timers[0].cancelled
