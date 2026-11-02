"""Offline owner evidence: replay status, fence stopped requests, never replay motion."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
import py_trees
from fault_detector_msgs.msg import CommandStatus
from fault_detector_msgs.srv import GetCommandStatus
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.command_status_ledger import CommandStatusLedger
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.behaviour_tree.behaviours.confirm_robot_stop import ConfirmRobotStop
from fault_detector_spot.application.ros.command_request_adapter import command_request_to_message
from fault_detector_spot.application.commanding.command_ids import CommandID
from test_command_request_correlation import semantic_request
from test_bt_startup import ConsumerNode
from test_buffer_and_status_publisher import behavior


def consumer():
    result = CommandSubscriber()
    result.node = ConsumerNode()
    result.blackboard = SimpleNamespace(command_buffer=[], estop_flag=False)
    result.activate()
    return result


def wire(request, owner):
    instance = owner.status_ledger.execution_instance_id
    request = replace(request, execution_instance_id=request.execution_instance_id or instance,
                      stop_execution_instance_ids=tuple(
                          item or instance for item in
                          (request.stop_execution_instance_ids or ("",) * len(request.stop_request_ids))
                      ))
    return command_request_to_message(request)


def stop_request(*targets):
    return replace(semantic_request(CommandID.EMERGENCY_CANCEL, "operator"),
                   stop_request_ids=tuple(targets))


def read_status(owner, request_id, expected_instance=None):
    expected = owner.status_ledger.execution_instance_id if expected_instance is None else expected_instance
    return owner.query_status(GetCommandStatus.Request(request_id=request_id, execution_instance_id=expected),
                              GetCommandStatus.Response()).status


def test_duplicate_receipt_and_query_do_not_buffer_or_execute_twice():
    owner = consumer()
    request = semantic_request(CommandID.STAND_UP, "operator")
    message = wire(request, owner)
    owner.append_request_to_buffer(message)
    owner.append_request_to_buffer(message)
    assert len(owner.pending_msgs) == 1
    owner.update()
    assert len(owner.blackboard.command_buffer) == 1
    owner.append_request_to_buffer(message)
    status = read_status(owner, request.request_id)
    assert status.state == CommandStatus.STATE_RUNNING
    assert not status.execution_idle
    assert status.buffered_command_count == 1
    assert not owner.pending_msgs
    assert len(owner.blackboard.command_buffer) == 1


@pytest.mark.parametrize("state", [CommandStatus.STATE_RUNNING, CommandStatus.STATE_UNKNOWN])
def test_malformed_duplicate_replays_existing_evidence_without_false_failure(state):
    owner = consumer()
    request = semantic_request(CommandID.STAND_UP, "operator")
    message = wire(request, owner)
    owner.append_request_to_buffer(message)
    status = read_status(owner, request.request_id)
    status.state = state
    owner.status_ledger.record(status)
    message.client_id = ""
    owner.append_request_to_buffer(message)
    replay = owner.request_status_publisher.messages[-1]
    assert replay.request_id == request.request_id
    assert replay.state == state
    assert not replay.execution_idle
    assert len(owner.pending_msgs) == 1


def test_rejected_new_envelope_is_correlated_nonexecution_evidence():
    owner = consumer()
    request = semantic_request(CommandID.STAND_UP, "operator")
    message = wire(request, owner)
    message.client_id = ""
    owner.append_request_to_buffer(message)
    status = read_status(owner, request.request_id)
    assert status.state == CommandStatus.STATE_FAILED
    assert status.execution_idle
    assert not owner.pending_msgs
    assert not owner.blackboard.command_buffer


def test_unknown_request_or_restarted_owner_never_claims_idle():
    owner = consumer()
    request = semantic_request(CommandID.STAND_UP, "operator")
    owner.append_request_to_buffer(wire(request, owner))
    restarted = consumer()
    status = read_status(restarted, request.request_id)
    assert status.state == CommandStatus.STATE_UNKNOWN
    assert not status.execution_idle
    assert not restarted.pending_msgs


def test_lost_terminal_delivery_is_recoverable_by_query_and_duplicate():
    owner = consumer()
    request = semantic_request(CommandID.WAIT_TIME, "operator")
    owner.append_request_to_buffer(wire(request, owner))
    publisher = behavior(Status.SUCCESS, request.command.command_id.value)
    publisher.status_ledger = owner.status_ledger
    publisher.execution_idle = lambda: True
    publisher.blackboard.last_command.request_id = request.request_id
    publisher.update()  # Drop this publication as if transport lost it.
    owner.append_request_to_buffer(wire(request, owner))
    status = read_status(owner, request.request_id)
    assert status.state == CommandStatus.STATE_SUCCEEDED and status.execution_idle
    replay = owner.request_status_publisher.messages[-1]
    assert replay.request_id == status.request_id and replay.state == status.state
    assert replay.execution_idle and replay.execution_instance_id == status.execution_instance_id
    assert replay.status_sequence < status.status_sequence
    status.execution_idle = False
    assert read_status(owner, request.request_id).execution_idle  # Query returns a snapshot.


def test_terminal_tree_status_waits_for_owner_evidence_and_keeps_original_outcome():
    publisher = behavior(Status.SUCCESS)
    idle = False
    publisher.execution_idle = lambda: idle
    publisher.update()
    first = publisher.structured_status_pub.messages[-1]
    assert first.state == CommandStatus.STATE_SUCCEEDED and not first.execution_idle
    publisher.blackboard.command_tree_status = Status.FAILURE  # Idle guard after completion.
    idle = True
    publisher.update()
    last = publisher.structured_status_pub.messages[-1]
    assert last.state == CommandStatus.STATE_SUCCEEDED and last.execution_idle
    assert len(publisher.structured_status_pub.messages) == 2


def stop_publisher(owner, request):
    publisher = behavior(Status.SUCCESS, CommandID.EMERGENCY_CANCEL.value)
    publisher.status_ledger = owner.status_ledger
    publisher.blackboard = owner.blackboard
    publisher.blackboard.last_command = owner.blackboard.command_buffer.pop()
    publisher.blackboard.last_command.stop_confirmed = True
    publisher.blackboard.command_tree_status = Status.SUCCESS
    publisher.execution_idle = lambda: True
    publisher.stop_runtime_idle = lambda: True
    return publisher


def test_stop_fences_delayed_motion_and_confirms_all_interrupted_identities():
    owner = consumer()
    received = semantic_request(CommandID.STAND_UP, "operator")
    delayed = semantic_request(CommandID.STAND_UP, "operator")
    owner.append_request_to_buffer(wire(received, owner))
    stop = stop_request(received.request_id, delayed.request_id)
    owner.append_request_to_buffer(wire(stop, owner))
    assert not owner.pending_msgs
    owner.append_request_to_buffer(wire(delayed, owner))
    assert not owner.pending_msgs
    assert read_status(owner, delayed.request_id).state == CommandStatus.STATE_STOP_REQUESTED
    publisher = stop_publisher(owner, stop)
    publisher.update()
    assert {m.request_id for m in publisher.structured_status_pub.messages} == {
        received.request_id, delayed.request_id, stop.request_id,
    }
    for request in (received, delayed, stop):
        status = read_status(owner, request.request_id)
        assert status.state == CommandStatus.STATE_STOP_CONFIRMED and status.execution_idle
        owner.append_request_to_buffer(wire(request, owner))
    assert not owner.pending_msgs and not owner.blackboard.command_buffer


@pytest.mark.parametrize("missing_proof", ["motion", "runtime", "physical_stop"])
def test_emergency_never_infers_confirmation_from_tree_success(missing_proof):
    owner = consumer()
    request = stop_request()
    owner.append_request_to_buffer(wire(request, owner))
    publisher = stop_publisher(owner, request)
    if missing_proof == "motion":
        publisher.execution_idle = lambda: False
    elif missing_proof == "runtime":
        publisher.stop_runtime_idle = lambda: False
    else:
        publisher.blackboard.last_command.stop_confirmed = False
    publisher.update()
    status = read_status(owner, request.request_id)
    assert status.state != CommandStatus.STATE_STOP_CONFIRMED
    assert not status.execution_idle


def test_previous_stop_cannot_confirm_a_new_stop_buffered_during_its_completion():
    owner = consumer()
    first = stop_request()
    owner.append_request_to_buffer(wire(first, owner))
    publisher = stop_publisher(owner, first)
    second = stop_request(first.request_id)
    owner.append_request_to_buffer(wire(second, owner))
    publisher.update()
    assert read_status(owner, second.request_id).state == CommandStatus.STATE_STOP_REQUESTED
    assert not read_status(owner, first.request_id).execution_idle
    assert len(owner.blackboard.command_buffer) == 1
    publisher.blackboard.last_command = owner.blackboard.command_buffer.pop()
    publisher.blackboard.last_command.stop_confirmed = True
    publisher.update()
    assert read_status(owner, first.request_id).execution_idle
    assert read_status(owner, second.request_id).execution_idle


def test_cached_previous_stop_confirmation_cannot_release_a_new_stop():
    ledger = CommandStatusLedger()
    first = replace(stop_request(), execution_instance_id=ledger.execution_instance_id)
    ledger.request_stop(first)
    proof = CommandStatus(request_id=first.request_id,
                          state=CommandStatus.STATE_STOP_CONFIRMED, execution_idle=True)
    ledger.confirm_stop(proof)
    second = replace(stop_request(), execution_instance_id=ledger.execution_instance_id)
    ledger.request_stop(second)
    assert ledger.confirm_stop(proof) == []
    assert not ledger.query(second.request_id).execution_idle
    ordinary = semantic_request(CommandID.STAND_UP, "operator")
    accepted, result = ledger.receive(replace(ordinary, execution_instance_id=ledger.execution_instance_id))
    assert not accepted and result.execution_idle  # Never entered the buffer.


def test_stop_timeout_retains_running_guard_then_accepts_late_physical_confirmation():
    now = [0.0]
    stopped = [False]
    starts = []
    resources = SimpleNamespace(
        begin_stop_confirmation=lambda node: starts.append(node),
        poll_stop_confirmation=lambda: (stopped[0], "Stop measurement"),
    )
    leaf = ConfirmRobotStop(resources, timeout_sec=1.0, monotonic_clock=lambda: now[0])
    command = SimpleNamespace()
    leaf.blackboard = SimpleNamespace(last_command=command)
    leaf.setup(node=object())
    leaf.initialise()
    assert leaf.update() is Status.RUNNING
    now[0] = 2.0
    assert leaf.update() is Status.RUNNING and command.stop_unconfirmed
    assert not command.stop_confirmed
    stopped[0] = True
    assert leaf.update() is Status.SUCCESS and command.stop_confirmed
    assert not command.stop_unconfirmed and len(starts) == 1


@pytest.mark.parametrize("recovery_allowed", [False, True])
def test_emergency_tree_proves_stop_before_and_after_recovery_motion(monkeypatch, recovery_allowed):
    from fault_detector_spot.application.behaviour_tree import runner

    monkeypatch.setattr(py_trees.blackboard.Blackboard, "storage", {})
    command = SimpleNamespace(command_id=CommandID.EMERGENCY_CANCEL, stop_recovery_allowed=recovery_allowed)
    py_trees.blackboard.Blackboard.set("last_command", command)
    py_trees.blackboard.Blackboard.set("estop_flag", True)
    events = []
    confirmations = iter([False, True, False, True])
    resources = SimpleNamespace(
        begin_stop_confirmation=lambda node: events.append("confirm"),
        poll_stop_confirmation=lambda: (next(confirmations), "measured stop"),
    )
    helpers = SimpleNamespace(robot_command_resources=resources, rtabmap_runtime=object())
    monkeypatch.setattr(runner, "get_helper_container", lambda node: helpers)

    class Completed(py_trees.behaviour.Behaviour):
        def update(self):
            events.append(self.name)
            return Status.SUCCESS

    monkeypatch.setattr(runner, "match_command_checker", lambda command_id: Completed("match"))
    monkeypatch.setattr(runner, "PublishZeroVel", lambda **kwargs: Completed("zero_velocity"))
    monkeypatch.setattr(runner, "StopMapping", lambda *args, **kwargs: Completed("stop_mapping"))
    monkeypatch.setattr(runner, "StowArmBehaviour", lambda **kwargs: Completed("stow"))
    monkeypatch.setattr(runner, "CloseGripperAction", lambda **kwargs: Completed("close"))
    monkeypatch.setattr(runner, "build_command_tree", lambda node: Completed("normal"))
    tree = runner.build_cancelable_command_tree(object())
    sequence = tree.decorated.children[0]
    for leaf in sequence.children:
        leaf.setup(node=object())
    sequence.tick_once()
    if not recovery_allowed:
        assert sequence.status is Status.FAILURE
        assert events == ["match"]
        assert py_trees.blackboard.Blackboard.get("estop_flag")
        return
    assert sequence.status is Status.RUNNING
    assert events == ["match", "zero_velocity", "confirm"]
    sequence.tick_once()
    assert sequence.status is Status.RUNNING
    assert events == ["match", "zero_velocity", "confirm", "stop_mapping", "stow", "close", "confirm"]
    assert py_trees.blackboard.Blackboard.get("estop_flag")
    assert not command.stop_confirmed
    sequence.tick_once()
    assert sequence.status is Status.SUCCESS
    assert command.stop_confirmed
    assert not py_trees.blackboard.Blackboard.get("estop_flag")
