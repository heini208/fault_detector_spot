"""Execution-instance fences and query heartbeats across a BT owner restart."""

from dataclasses import replace

import pytest
from fault_detector_msgs.msg import CommandStatus
from fault_detector_msgs.srv import GetCommandStatus
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from test_buffer_and_status_publisher import behavior
from test_command_request_correlation import semantic_request
from test_command_owner_reconciliation import consumer, wire, read_status, stop_request, stop_publisher


def query(owner, request_id="", expected=""):
    return owner.query_status(
        GetCommandStatus.Request(request_id=request_id, execution_instance_id=expected),
        GetCommandStatus.Response(),
    )


def test_discovery_is_read_only_and_reports_activation_and_occupancy():
    inactive = CommandSubscriber()
    assert not query(inactive).accepting_commands
    owner = consumer()
    first = query(owner)
    assert first.accepting_commands and first.unresolved_request_ids == []
    assert first.status.execution_instance_id == owner.status_ledger.execution_instance_id
    assert not first.status.execution_idle
    request = semantic_request(CommandID.STAND_UP, "operator")
    owner.append_request_to_buffer(wire(request, owner))
    busy = query(owner)
    assert not busy.accepting_commands
    assert busy.unresolved_request_ids == [request.request_id]
    assert len(owner.pending_msgs) == 1
    for _ in range(3):
        unknown = query(owner, semantic_request(CommandID.WAIT_TIME, "operator").request_id,
                        owner.status_ledger.execution_instance_id)
        assert unknown.status.state == CommandStatus.STATE_UNKNOWN
        assert not unknown.status.execution_idle
        assert unknown.unresolved_request_ids == [request.request_id]
    assert len(owner.pending_msgs) == 1 and not owner.blackboard.command_buffer


@pytest.mark.parametrize("target", ["", "prior-owner"])
@pytest.mark.parametrize("command_id", [CommandID.STAND_UP, CommandID.EMERGENCY_CANCEL])
def test_stale_or_unbound_dispatch_is_rejected_before_parse_or_cache(target, command_id):
    owner = consumer()
    request = semantic_request(command_id, "operator")
    message = wire(request, owner)
    message.execution_instance_id = target
    message.client_id = ""  # Invalid payload cannot turn stale delivery into idle proof.
    owner.append_request_to_buffer(message)
    result = owner.request_status_publisher.messages[-1]
    assert result.state == CommandStatus.STATE_UNKNOWN
    assert result.execution_instance_id == owner.status_ledger.execution_instance_id
    assert result.status_sequence > 0 and not result.execution_idle
    assert not owner.status_ledger.contains(request.request_id)
    assert not owner.pending_msgs and not owner.blackboard.command_buffer
    assert not owner.blackboard.estop_flag


def test_restart_never_adopts_delayed_dispatch_or_reports_old_request_idle():
    previous = consumer()
    request = semantic_request(CommandID.STAND_UP, "operator")
    message = wire(request, previous)
    previous.append_request_to_buffer(message)
    restarted = consumer()
    assert previous.status_ledger.execution_instance_id != restarted.status_ledger.execution_instance_id
    restarted.append_request_to_buffer(message)
    result = query(restarted, request.request_id, previous.status_ledger.execution_instance_id)
    assert result.status.execution_instance_id == restarted.status_ledger.execution_instance_id
    assert result.status.state == CommandStatus.STATE_UNKNOWN and not result.status.execution_idle
    assert result.unresolved_request_ids == ["shutdown:bt_runner/active_session"]
    assert not result.accepting_commands
    assert not restarted.pending_msgs and not restarted.blackboard.command_buffer


def test_mismatched_query_cannot_reuse_a_matching_request_id_terminal_snapshot():
    owner = consumer()
    request = semantic_request(CommandID.WAIT_TIME, "operator")
    owner.append_request_to_buffer(wire(request, owner))
    terminal = CommandStatus(request_id=request.request_id, state=CommandStatus.STATE_SUCCEEDED,
                             execution_idle=True)
    owner.status_ledger.record(terminal)
    for wrong_instance in ("", "prior-owner"):
        result = query(owner, request.request_id, wrong_instance)
        assert result.status.state == CommandStatus.STATE_UNKNOWN
        assert not result.status.execution_idle
    assert read_status(owner, request.request_id).execution_idle


def test_fresh_queries_and_actual_emissions_advance_sequence_without_periodic_republication():
    owner = consumer()
    request = semantic_request(CommandID.WAIT_TIME, "operator")
    owner.append_request_to_buffer(wire(request, owner))
    receipt = owner.request_status_publisher.messages[-1]
    publisher = behavior(Status.SUCCESS, request.command.command_id.value)
    publisher.status_ledger = owner.status_ledger
    publisher.execution_idle = lambda: True
    publisher.blackboard.last_command.request_id = request.request_id
    publisher.update()
    completion = publisher.structured_status_pub.messages[-1]
    assert completion.status_sequence == receipt.status_sequence + 1
    heartbeat = query(owner)
    assert heartbeat.status.status_sequence == completion.status_sequence + 1
    for _ in range(5):
        publisher.update()
    following = query(owner)
    assert following.status.status_sequence == heartbeat.status.status_sequence + 1
    assert len(publisher.structured_status_pub.messages) == 1
    assert following.status.execution_instance_id == receipt.execution_instance_id


def test_foreign_stop_target_blocks_recovery_and_cannot_be_confirmed_by_new_empty_owner():
    owner = consumer()
    current = semantic_request(CommandID.STAND_UP, "operator")
    prior = semantic_request(CommandID.STAND_UP, "operator")
    stop = replace(stop_request(current.request_id, prior.request_id),
                   stop_execution_instance_ids=(owner.status_ledger.execution_instance_id, "prior-owner"))
    owner.append_request_to_buffer(wire(stop, owner))
    command = owner.blackboard.command_buffer[0]
    assert not command.stop_confirmation_allowed and command.stop_unconfirmed
    assert owner.request_status_publisher.messages[-1].state == CommandStatus.STATE_STOP_UNCONFIRMED
    owner.append_request_to_buffer(wire(current, owner))
    assert not owner.pending_msgs and owner.blackboard.command_buffer == [command]
    publisher = stop_publisher(owner, stop)
    publisher.update()  # Even an erroneous tree-success snapshot cannot override the fence.
    result = query(owner)
    assert not result.accepting_commands
    assert set(result.unresolved_request_ids) == {current.request_id, prior.request_id, stop.request_id}
    for request in (current, prior, stop):
        status = read_status(owner, request.request_id)
        assert status.state == CommandStatus.STATE_STOP_UNCONFIRMED and not status.execution_idle
    following_stop = stop_request()
    owner.append_request_to_buffer(wire(following_stop, owner))
    assert not owner.blackboard.command_buffer[0].stop_confirmation_allowed


@pytest.mark.parametrize("completed", [False, True])
def test_duplicate_stop_identity_never_restarts_recovery(completed):
    owner = consumer()
    request = stop_request()
    message = wire(request, owner)
    owner.append_request_to_buffer(message)
    command = owner.blackboard.command_buffer[0]
    if completed:
        publisher = stop_publisher(owner, request)
        publisher.update()
    for _ in range(3):
        owner.append_request_to_buffer(message)
    assert owner.blackboard.command_buffer == ([] if completed else [command])
    assert not owner.pending_msgs
    state = CommandStatus.STATE_STOP_CONFIRMED if completed else CommandStatus.STATE_STOP_REQUESTED
    assert owner.request_status_publisher.messages[-1].state == state
