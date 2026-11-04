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
          idle=False, accepting=False, unresolved=None, latched=False, confirmed=False):
    request, future = query or p.node.status_client.calls[-1]
    sequence = p.sequences.get(instance, 0) + 1
    p.sequences[instance] = sequence
    status = CommandStatus(request_id=request.request_id, execution_instance_id=instance,
                           status_sequence=sequence, state=state, execution_idle=idle,
                           software_stop_latched=latched, software_stop_confirmed=confirmed)
    future.set_result(SimpleNamespace(status=status, accepting_commands=accepting,
                                     unresolved_request_ids=(
                                         [] if accepting else [request.request_id]
                                     ) if unresolved is None else unresolved))
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


@pytest.mark.parametrize("unresolved", [[], ["shutdown:bt_runner/arm/default"]])
def test_configuration_can_change_under_stop_with_only_shutdown_markers(unresolved):
    p = rig()
    reply(p, unresolved=unresolved, latched=True)
    stop_id = p.controller.software_stop_status.request_id
    assert stop_id == ""
    assert p.controller.queued_request_ids == ()
    p.controller.poll()
    p.transport._query_status("")
    reply(p, unresolved=unresolved, latched=True)

    assert p.controller.run_configuration_change(lambda: "changed") == "changed"
    assert p.controller.software_stop_status.latched
    assert not p.controller.software_stop_status.stop_confirmed
    with pytest.raises(RuntimeError):
        p.controller.run_if_idle(lambda: pytest.fail("Stop admitted a workflow"))
    assert p.wire == []


def test_configuration_blocks_remote_work_unknown_to_local_controller():
    p = rig()
    foreign_request = make_request(CommandID.READY_ARM)
    reply(p, unresolved=[foreign_request.request_id], latched=True)

    assert p.controller.queued_request_ids == ()
    with pytest.raises(RuntimeError):
        p.controller.run_configuration_change(lambda: pytest.fail("Remote motion admitted configuration"))


@pytest.mark.parametrize("owner_evidence", ["missing", "expired_query", "stale_snapshot"])
def test_configuration_under_stop_requires_current_owner_query(owner_evidence):
    p = rig()
    p.controller.cancel_all()
    if owner_evidence == "expired_query":
        p.now[0] = 2.0
        reply(p, unresolved=[])
    elif owner_evidence == "stale_snapshot":
        reply(p, unresolved=[])
        p.now[0] = 5.0

    with pytest.raises(RuntimeError):
        p.controller.run_configuration_change(lambda: pytest.fail("Missing owner proof admitted configuration"))


def test_newer_topic_invalidates_configuration_occupancy_until_newer_query():
    p = rig()
    reply(p, unresolved=[], latched=True)
    assert p.controller.run_configuration_change(lambda: True)
    p.transport._query_status("")
    older_query = p.node.status_client.calls[-1]
    foreign_request = make_request(CommandID.READY_ARM)
    topic = CommandStatus(
        request_id=foreign_request.request_id, execution_instance_id="owner-a",
        status_sequence=p.sequences["owner-a"] + 2, state=CommandStatus.STATE_RUNNING,
    )
    p.transport.handle_command_status(topic)
    # A later-arriving query generated before this topic cannot restore an old
    # empty occupancy snapshot, even though it still proves owner liveness.
    reply(p, older_query, unresolved=[], latched=True)
    with pytest.raises(RuntimeError):
        p.controller.run_configuration_change(lambda: pytest.fail("Stale occupancy admitted configuration"))

    p.sequences["owner-a"] += 1
    p.transport._query_status("")
    reply(p, unresolved=[], latched=True)
    assert p.controller.run_configuration_change(lambda: True)


@pytest.mark.parametrize("heartbeat", ["stop", "shutdown_marker"])
def test_stop_heartbeats_do_not_starve_configuration_but_queries_must_stay_fresh(heartbeat):
    p = rig()
    reply(p, unresolved=[], latched=True)
    stop_id = p.controller.cancel_all()
    heartbeat_id = stop_id if heartbeat == "stop" else "shutdown:bt_runner/arm/default"
    for second in range(1, 10):
        p.now[0] = float(second)
        p.transport._query_status("")
        topic = CommandStatus(
            request_id=heartbeat_id, execution_instance_id="owner-a",
            status_sequence=p.sequences["owner-a"] + 2,
            state=CommandStatus.STATE_STOP_UNCONFIRMED, software_stop_latched=True,
        )
        p.transport.handle_command_status(topic)
        # This valid full query was generated before the newer stop heartbeat;
        # the heartbeat owns no motion and must not prevent its occupancy update.
        reply(p, unresolved=[stop_id, "shutdown:bt_runner/arm/default"], latched=True)
        p.sequences["owner-a"] += 1
        assert p.controller.run_configuration_change(lambda: True)

    p.now[0] += 5.0
    topic.status_sequence += 1
    p.transport.handle_command_status(topic)
    with pytest.raises(RuntimeError):
        p.controller.run_configuration_change(lambda: pytest.fail("Heartbeat replaced a fresh query"))


def test_owner_restart_cannot_reuse_prior_configuration_occupancy():
    p = rig()
    reply(p, unresolved=[], latched=True)
    assert p.controller.run_configuration_change(lambda: True)
    stop_id = p.controller.cancel_all()
    p.transport._query_status(stop_id)
    previous_owner_query = p.node.status_client.calls[-1]
    p.transport._query_status("")
    foreign_request = make_request(CommandID.READY_ARM)
    reply(p, instance="owner-b", unresolved=[foreign_request.request_id], latched=True)
    reply(p, previous_owner_query, unresolved=[], latched=True)

    with pytest.raises(RuntimeError):
        p.controller.run_configuration_change(lambda: pytest.fail("Restart hid remote motion"))
