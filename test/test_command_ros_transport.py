"""Tests for the ROS adapter around the pure command controller."""

from types import SimpleNamespace
from concurrent.futures import Future
from itertools import count

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


OWNER_INSTANCE = "test-bt-owner"
_sequences = count(1)


def owner_status(**kwargs):
    return CommandStatus(execution_instance_id=OWNER_INSTANCE,
                         status_sequence=next(_sequences), **kwargs)


def query_response(status, *, accepting=None):
    return SimpleNamespace(status=status,
                           accepting_commands=status.execution_idle if accepting is None else accepting,
                           unresolved_request_ids=[] if status.execution_idle or accepting else [status.request_id])


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


class FakeStatusClient:
    def __init__(self, auto_discovery=True):
        self.auto_discovery = auto_discovery
        self.ready = True
        self.calls = []
        self.removed = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        if not request.request_id and self.auto_discovery:
            future.set_result(query_response(owner_status(), accepting=True))
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeNode:
    def __init__(self, auto_discovery=True):
        self.publishers = {}
        self.subscriptions = {}
        self.timers = []
        self.logger = FakeLogger()
        self.status_client = FakeStatusClient(auto_discovery)

    def create_client(self, _service_type, _service_name):
        return self.status_client

    def destroy_client(self, _client):
        pass

    def create_publisher(self, _message_type, topic, _qos):
        count = 1 if topic == "fault_detector/_internal/commands/request" else 0
        publisher = FakePublisher(subscription_count=count)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_timer(self, _period, callback, *, clock):
        timer = FakeTimer(callback)
        timer.clock = clock
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
    assert message.header.stamp == Time(
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

    running = owner_status()
    running.request_id = first.request_id
    running.state = CommandStatus.STATE_RUNNING
    running.buffered_command_count = 1
    node.subscriptions[
        "fault_detector/_internal/command_status"
    ](running)

    assert states[-1].state is CommandControllerState.RUNNING
    assert controller.active_request_id == first.request_id

    succeeded = owner_status()
    succeeded.request_id = first.request_id
    succeeded.state = CommandStatus.STATE_SUCCEEDED
    succeeded.execution_idle = True
    succeeded.buffered_command_count = 0
    node.subscriptions[
        "fault_detector/_internal/command_status"
    ](succeeded)

    assert controller.active_request_id == second.request_id


def test_execution_retry_permission_survives_controller_transport():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.MOVE_TO_WAYPOINT)
    controller.submit(request)
    failed = owner_status(request_id=request.request_id,
                           state=CommandStatus.STATE_FAILED, retry_safe=True, execution_idle=True)
    node.subscriptions["fault_detector/_internal/command_status"](failed)
    status = node.publishers["fault_detector/_internal/commands/status"].messages[-1]
    assert status.state == CommandStatus.STATE_FAILED
    assert status.retry_safe


def test_safe_continuation_permission_survives_controller_transport():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.MOVE_TO_WAYPOINT)
    controller.submit(request)
    failed = owner_status(request_id=request.request_id,
                           state=CommandStatus.STATE_FAILED, continuation_safe=True, execution_idle=True)
    node.subscriptions["fault_detector/_internal/command_status"](failed)
    status = node.publishers["fault_detector/_internal/commands/status"].messages[-1]
    assert status.continuation_safe and not status.retry_safe


def test_invalid_ros_submission_is_rejected_before_queueing():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    request = make_request(CommandID.STAND_UP)
    message = command_request_to_message(request)
    message.payload.command_id = ""

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


def test_emergency_bypasses_local_command_handler():
    node = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node, controller)
    local_calls = []
    transport.set_local_dispatch(lambda request: local_calls.append(request) or True)

    request_id = controller.cancel_all()

    messages = node.publishers[
        "fault_detector/_internal/commands/request"
    ].messages
    assert [message.request_id for message in messages] == [request_id]
    assert local_calls == []
    controller.poll()
    assert controller.active_request_id == request_id


def test_emergency_publish_does_not_wait_for_active_request_completion():
    node = FakeNode()
    controller = CommandController()
    RosCommandTransport(node, controller)
    active = make_request(CommandID.MOVE_BASE_RELATIVE)

    controller.submit(active)

    dispatch = node.publishers[
        "fault_detector/_internal/commands/request"
    ]
    assert [message.request_id for message in dispatch.messages] == [
        active.request_id
    ]

    emergency_id = controller.cancel_all()

    assert [message.request_id for message in dispatch.messages] == [
        active.request_id,
        emergency_id,
    ]
    assert controller.active_request_id == active.request_id

    controller.poll()

    assert controller.active_request_id == active.request_id


def test_failure_replacement_survives_controller_transport_and_invalid_payload_blocks_retry():
    from fault_detector_spot.application.ros.semantic_command_adapter import (
        semantic_command_to_message, semantic_command_from_message,
    )

    for invalid in (False, True):
        node = FakeNode()
        controller = CommandController()
        transport = RosCommandTransport(node, controller)
        request = make_request(CommandID.WAIT_TIME)
        transport.submit_message(command_request_to_message(request))
        replacement = SemanticCommand(CommandID.WAIT_TIME, wait_time=3.0)
        failure = owner_status(request_id=request.request_id, state=CommandStatus.STATE_FAILED,
                                retry_safe=True, has_retry_command=True, execution_idle=True,
                                retry_command=semantic_command_to_message(replacement))
        if invalid:
            failure.retry_command.command_id = "unknown_command"
        assert transport.handle_command_status(failure)
        status = node.publishers["fault_detector/_internal/commands/status"].messages[-1]
        assert status.retry_safe is (not invalid)
        assert status.has_retry_command is (not invalid)
        if not invalid:
            assert semantic_command_from_message(status.retry_command) == replacement


def test_duplicate_submission_does_not_publish_false_terminal_feedback():
    node = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node, controller)
    request = make_request(CommandID.READY_ARM)
    message = command_request_to_message(request)
    assert transport.submit_message(message)
    statuses = node.publishers["fault_detector/_internal/commands/status"].messages
    count = len(statuses)
    assert not transport.submit_message(message)
    assert len(statuses) == count
    assert controller.active_request_id == request.request_id
    # Even a malformed duplicate cannot impersonate an execution failure.
    message.payload.command_id = ""
    assert not transport.submit_message(message)
    assert len(statuses) == count


def test_read_only_reconciliation_recovers_lost_terminal_without_resending():
    now = [0.0]
    node = FakeNode()
    controller = CommandController(monotonic_clock=lambda: now[0])
    RosCommandTransport(node, controller)
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)
    now[0] = 4.0
    controller.poll()
    query, future = node.status_client.calls[-1]
    assert query.request_id == first.request_id
    dispatch = node.publishers["fault_detector/_internal/commands/request"].messages
    assert len(dispatch) == 1
    future.set_result(query_response(owner_status(
        request_id=first.request_id, state=CommandStatus.STATE_SUCCEEDED, execution_idle=True,
    )))
    assert controller.active_request_id == second.request_id
    assert len(dispatch) == 2


def test_unavailable_query_and_missing_owner_history_retain_lane():
    now = [0.0]
    node = FakeNode()
    controller = CommandController(monotonic_clock=lambda: now[0])
    RosCommandTransport(node, controller, monotonic_clock=lambda: now[0])
    node.status_client.ready = False
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)
    now[0] = 4.0
    controller.poll()
    assert len(node.status_client.calls) == 1  # Initial owner discovery.
    assert controller.active_request_id == request.request_id
    node.status_client.ready = True
    now[0] = 8.0
    controller.poll()
    node.status_client.calls[-1][1].set_result(query_response(owner_status(
        request_id=request.request_id, state=CommandStatus.STATE_UNKNOWN,
    )))
    assert controller.active_request_id == request.request_id
    assert len(node.publishers["fault_detector/_internal/commands/request"].messages) == 1


def test_bt_stop_cannot_confirm_a_locally_owned_request():
    node = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node, controller)
    transport.set_local_dispatch(lambda request: True)
    request = make_request(CommandID.START_SENSOR_RECORDING)
    controller.submit(request)
    stop_id = controller.cancel_all()
    controller.poll()
    wire_stop = node.publishers["fault_detector/_internal/commands/request"].messages[-1]
    assert wire_stop.request_id == stop_id
    assert request.request_id not in wire_stop.stop_request_ids
    assert not transport.handle_command_status(owner_status(
        request_id=request.request_id, state=CommandStatus.STATE_STOP_CONFIRMED, execution_idle=True,
    ))
    assert controller.active_request_id == request.request_id


def test_query_failure_and_wrong_correlation_never_release_lane():
    node = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node, controller)
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)
    transport._query_status(request.request_id)
    node.status_client.calls[-1][1].set_exception(RuntimeError("owner disconnected"))
    assert controller.active_request_id == request.request_id

    transport._query_status(request.request_id)
    node.status_client.calls[-1][1].set_result(query_response(owner_status(
        request_id=make_request(CommandID.READY_ARM).request_id,
        state=CommandStatus.STATE_SUCCEEDED, execution_idle=True,
    )))
    assert controller.active_request_id == request.request_id


def test_timed_out_query_is_replaced_and_its_late_reply_is_ignored():
    now = [0.0]
    node = FakeNode()
    controller = CommandController()
    transport = RosCommandTransport(node, controller, monotonic_clock=lambda: now[0])
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)
    transport._query_status(request.request_id)
    old = node.status_client.calls[-1][1]
    now[0] = 4.0
    transport._query_status(request.request_id)
    assert node.status_client.removed == [old]
    assert len(node.status_client.calls) == 3
    old.set_result(query_response(owner_status(
        request_id=request.request_id, state=CommandStatus.STATE_SUCCEEDED, execution_idle=True,
    )))
    assert controller.active_request_id == request.request_id
