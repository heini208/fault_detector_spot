"""Fresh owner queries and local monotonic deadlines, with no live ROS nodes."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import CommandStatus
from rclpy.clock import ClockType

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandController, CommandControllerState, CommandExecutionUnresolved,
)
from fault_detector_spot.application.ros.command_transport import RosCommandTransport
from test_command_ros_transport import FakeNode, make_request


def rig():
    now, ros_time = [0.0], [0]
    node = FakeNode(auto_discovery=False)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=ros_time[0]),
    ))
    controller = CommandController(monotonic_clock=lambda: now[0])
    states = []
    controller.add_status_listener(states.append)
    transport = RosCommandTransport(node, controller, monotonic_clock=lambda: now[0])
    return SimpleNamespace(now=now, ros_time=ros_time, node=node, controller=controller,
                           transport=transport, states=states, sequences={},
                           wire=node.publishers['fault_detector/_internal/commands/request'].messages)


def reply(p, query=None, *, instance="owner-a", state=CommandStatus.STATE_RUNNING,
          idle=False, accepting=False):
    request, future = query or p.node.status_client.calls[-1]
    sequence = p.sequences.get(instance, 0) + 1
    p.sequences[instance] = sequence
    status = CommandStatus(request_id=request.request_id, execution_instance_id=instance,
                           status_sequence=sequence, state=state, execution_idle=idle)
    future.set_result(SimpleNamespace(status=status, accepting_commands=accepting,
                                     unresolved_request_ids=[] if accepting else [request.request_id]))
    return status


def discover(p):
    reply(p, accepting=True)


def test_discovery_requires_fresh_idle_owner_before_dispatch():
    p = rig()
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    assert p.wire == []
    reply(p, accepting=False)
    p.controller.poll()
    assert p.wire == []
    p.transport._query_status("")
    discover(p)
    p.controller.poll()
    assert [message.request_id for message in p.wire] == [request.request_id]
    assert p.wire[0].execution_instance_id == "owner-a"


def test_healthy_long_operation_and_ros_time_jumps_do_not_expire():
    p = rig()
    discover(p)
    request = make_request(CommandID.WAIT_TIME)
    p.controller.submit(request)
    assert p.node.timers[0].clock.clock_type == ClockType.STEADY_TIME
    for second in range(1, 101):
        p.now[0] = float(second)
        p.ros_time[0] = (0, 100000, 1, 0)[second % 4]
        p.node.timers[0].callback()
        for query in p.node.status_client.calls:
            if not query[1].done():
                reply(p, query)
        assert p.controller.active_request_id == request.request_id
        assert p.states[-1].state is CommandControllerState.RUNNING
    assert not any(status.state is CommandControllerState.UNKNOWN for status in p.states)
    assert len(p.wire) == 1


def test_loss_then_same_owner_reconnect_retains_lane_without_replay():
    p = rig()
    discover(p)
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    p.controller.submit(first)
    p.controller.submit(second)
    p.transport._query_status(first.request_id)
    reply(p)
    p.node.status_client.ready = False
    p.now[0] = 5.0
    p.ros_time[0] = 0  # Simulated ROS time can remain paused indefinitely.
    p.node.timers[0].callback()
    assert next(status for status in reversed(p.states) if status.request_id == first.request_id).state is CommandControllerState.UNKNOWN
    with pytest.raises(CommandExecutionUnresolved):
        p.controller.submit(make_request(CommandID.STAND_UP))
    assert p.controller.queued_request_ids == (second.request_id,)
    assert len(p.wire) == 1
    p.node.status_client.ready = True
    p.transport._query_status(first.request_id)
    reply(p)
    assert p.states[-1].state is CommandControllerState.RUNNING
    assert len(p.wire) == 1
    p.transport._query_status(first.request_id)
    reply(p, state=CommandStatus.STATE_SUCCEEDED, idle=True, accepting=True)
    assert [message.request_id for message in p.wire] == [first.request_id, second.request_id]


def test_expired_query_reply_cannot_renew_liveness_or_release_lane():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    reply(p)
    p.transport._query_status(request.request_id)
    delayed = p.node.status_client.calls[-1]
    p.now[0] = 5.0
    reply(p, delayed, state=CommandStatus.STATE_SUCCEEDED, idle=True, accepting=True)
    p.controller.poll()
    assert p.controller.active_request_id == request.request_id
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    assert len(p.wire) == 1


def test_topic_older_than_fresh_discovery_cannot_renew_request_liveness():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    original = reply(p)
    delayed = CommandStatus(request_id=request.request_id, execution_instance_id="owner-a",
                            status_sequence=original.status_sequence + 1,
                            state=CommandStatus.STATE_RUNNING)
    p.sequences["owner-a"] += 1  # The delayed message was emitted before discovery.
    p.transport._query_status("")
    reply(p)
    p.now[0] = 5.0
    assert not p.transport.handle_command_status(delayed)
    p.controller.poll()
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    assert len(p.wire) == 1


def test_restart_retires_old_instance_and_cannot_adopt_prior_request():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    reply(p)
    p.transport._query_status(request.request_id)
    old_query = p.node.status_client.calls[-1]
    p.transport._query_status("")
    reply(p, instance="owner-b", accepting=True)
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    assert "owner changed" in p.states[-1].detail
    old_terminal = reply(p, old_query, state=CommandStatus.STATE_SUCCEEDED, idle=True, accepting=True)
    assert not p.transport.handle_command_status(old_terminal)
    p.transport._query_status(request.request_id)
    assert p.node.status_client.calls[-1][0].execution_instance_id == "owner-a"
    reply(p, instance="owner-b", state=CommandStatus.STATE_SUCCEEDED, idle=True, accepting=True)
    assert p.controller.active_request_id == request.request_id
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    with pytest.raises(CommandExecutionUnresolved):
        p.controller.submit(make_request(CommandID.STOW_ARM))
    stop = p.controller.cancel_all()
    assert p.controller.cancel_all() == stop
    assert len(p.wire) == 2
    assert p.wire[-1].execution_instance_id == "owner-b"
    assert list(p.wire[-1].stop_request_ids) == [request.request_id]
    assert list(p.wire[-1].stop_execution_instance_ids) == ["owner-a"]


def test_local_reconciliation_queries_local_owner_without_remote_replay():
    p = rig()
    discover(p)
    local_calls = []

    def dispatch(request):
        p.controller.bind_execution_owner(request.request_id, "local-owner")
        return True

    p.transport.set_local_dispatch(dispatch, query_status=local_calls.append)
    request = make_request(CommandID.START_SENSOR_RECORDING)
    p.controller.submit(request)
    p.now[0] = 1.0
    p.controller.poll()
    assert local_calls == [request.request_id]
    assert p.wire == []
    assert all(not query.request_id for query, _ in p.node.status_client.calls)


def test_intermediate_restart_reply_cannot_retire_newer_observed_owner():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    intermediate = p.node.status_client.calls[-1]
    p.transport._query_status("")
    reply(p, instance="owner-c", accepting=True)
    reply(p, intermediate, instance="owner-b", state=CommandStatus.STATE_UNKNOWN)
    p.transport._query_status("")
    reply(p, instance="owner-c", accepting=True)
    stop_id = p.controller.cancel_all()
    assert p.wire[-1].request_id == stop_id
    assert p.wire[-1].execution_instance_id == "owner-c"
    assert p.controller.active_request_id == request.request_id


def test_newer_topic_does_not_starve_fresh_query_liveness_for_stopping():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    for second in range(1, 10):
        p.now[0] = float(second)
        p.transport._query_status(request.request_id)
        query_sequence = p.sequences["owner-a"] + 1
        topic = CommandStatus(request_id=request.request_id, execution_instance_id="owner-a",
                              status_sequence=query_sequence + 1, state=CommandStatus.STATE_RUNNING)
        assert p.transport.handle_command_status(topic)
        reply(p)
        p.sequences["owner-a"] += 1
    stop_id = p.controller.cancel_all()
    assert p.wire[-1].request_id == stop_id
    assert len(p.wire) == 2


def test_discovery_client_exception_cannot_disable_controller_deadlines():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    reply(p)

    def disconnected(_request):
        raise RuntimeError("Client disconnected")

    p.node.status_client.call_async = disconnected
    p.now[0] = 5.0
    p.node.timers[0].callback()
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    assert p.node.logger.error_messages
    assert len(p.wire) == 1


def test_query_loss_does_not_prevent_explicit_stop_to_known_owner():
    p = rig()
    discover(p)
    request = make_request(CommandID.READY_ARM)
    p.controller.submit(request)
    p.transport._query_status(request.request_id)
    reply(p)
    p.node.status_client.ready = False
    p.now[0] = 6.0
    p.node.timers[0].callback()
    assert p.states[-1].state is CommandControllerState.UNKNOWN
    stop = p.controller.cancel_all()
    assert p.wire[-1].request_id == stop
    assert p.wire[-1].execution_instance_id == "owner-a"
    assert list(p.wire[-1].stop_execution_instance_ids) == ["owner-a"]
    p.now[0] = 10.0
    p.node.timers[0].callback()
    assert any(status.request_id == stop and status.state is CommandControllerState.STOP_UNCONFIRMED
               for status in p.states)
    assert p.controller.cancel_all() == stop
    assert p.controller.active_request_id == request.request_id
    assert len(p.wire) == 2
