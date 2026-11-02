"""Shutdown uncertainty survives owner restarts and fences command admission."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import CommandStatus
from fault_detector_msgs.srv import GetCommandStatus

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from test_bt_startup import ConsumerNode
from test_command_owner_reconciliation import wire, stop_request
from test_command_request_correlation import semantic_request


def owner(store, resources=None):
    subscriber = CommandSubscriber(shutdown_state_store=store, robot_command_resources=resources)
    subscriber.node = ConsumerNode()
    subscriber.blackboard = SimpleNamespace(command_buffer=[], estop_flag=False)
    subscriber.activate()
    return subscriber


def query(subscriber, request_id=""):
    return subscriber.query_status(
        GetCommandStatus.Request(request_id=request_id,
                                 execution_instance_id=subscriber.status_ledger.execution_instance_id),
        GetCommandStatus.Response(),
    )


def test_restart_fences_individual_resources_from_both_process_owners(tmp_path):
    bt_store = StopUnconfirmedStore("bt_runner", tmp_path)
    api_store = StopUnconfirmedStore("application_api", tmp_path)
    bt_store.record({"arm": "terminal result missing", "base": "fresh velocity missing"})
    api_store.record({"acquisition": "cancellation result missing"})
    latches = []
    restarted = owner(bt_store, SimpleNamespace(latch_software_stop=lambda: latches.append(True)))
    discovery = query(restarted)
    assert latches == [True]
    assert not discovery.accepting_commands
    assert discovery.status.state == CommandStatus.STATE_STOP_UNCONFIRMED
    assert discovery.status.software_stop_latched
    assert not discovery.status.software_stop_confirmed
    assert set(discovery.unresolved_request_ids) == {
        "shutdown:bt_runner/arm", "shutdown:bt_runner/base",
        "shutdown:application_api/acquisition",
    }
    for command in (CommandID.READY_ARM, CommandID.RESET_SOFTWARE_STOP):
        request = semantic_request(command, "operator")
        restarted.append_request_to_buffer(wire(request, restarted))
        assert restarted.request_status_publisher.messages[-1].state == CommandStatus.STATE_FAILED
    assert not restarted.pending_msgs and not restarted.blackboard.command_buffer
    # Empty new owners cannot certify prior action termination, even on a later close.
    restarted.report_shutdown_result({})
    assert not query(restarted).status.software_stop_confirmed
    assert len(bt_store.load()) == 3
    restarted.report_shutdown_result({
        f"prior shutdown/{name}": detail for name, detail in bt_store.load().items()
    })
    assert len(bt_store.load()) == 3
    assert not query(owner(bt_store)).accepting_commands


def test_close_retains_status_callbacks_and_reports_only_coordinator_proof(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    subscription, service = subscriber.request_subscription, subscriber.status_query_service
    subscriber.close_admission()
    subscriber.close_admission()
    subscriber.activate()
    assert not query(subscriber).accepting_commands
    assert not subscriber.pending_msgs and not subscriber.blackboard.command_buffer
    assert subscriber.request_subscription is subscription
    assert subscriber.status_query_service is service
    waiting = query(subscriber, request.request_id).status
    assert waiting.state == CommandStatus.STATE_STOP_REQUESTED and not waiting.execution_idle
    rejected = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(rejected, subscriber))
    assert query(subscriber, rejected.request_id).status.state == CommandStatus.STATE_FAILED
    subscriber.report_shutdown_result({"arm": "late goal acceptance remains unresolved"})
    missing = query(subscriber, request.request_id).status
    assert missing.state == CommandStatus.STATE_STOP_UNCONFIRMED and not missing.execution_idle
    subscriber.close_admission()
    assert query(subscriber, request.request_id).status.state == CommandStatus.STATE_STOP_UNCONFIRMED
    assert store.load() == {"bt_runner/arm": "late goal acceptance remains unresolved"}


def test_confirmed_close_preserves_closed_admission(tmp_path):
    subscriber = owner(StopUnconfirmedStore(root=tmp_path))
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    subscriber.close_admission()
    subscriber.report_shutdown_result({})
    status = query(subscriber, request.request_id).status
    assert status.state == CommandStatus.STATE_STOP_CONFIRMED and status.execution_idle
    assert status.software_stop_confirmed
    assert not query(subscriber).accepting_commands


def test_corrupt_marker_fails_closed_and_reconciliation_is_resource_specific(tmp_path):
    bt_store = StopUnconfirmedStore("bt_runner", tmp_path)
    api_store = StopUnconfirmedStore("application_api", tmp_path)
    bt_store.record({"arm": "result missing", "base": "telemetry missing"})
    api_store.record({"setup": "cancel pending"})
    bt_store.clear_confirmed(["arm"])
    assert bt_store.load() == {"bt_runner/base": "telemetry missing",
                               "application_api/setup": "cancel pending"}
    bt_store.path.write_text("not valid json", encoding="utf-8")
    restarted = owner(bt_store)
    assert not query(restarted).accepting_commands
    assert "shutdown:bt_runner/state_read_error" in query(restarted).unresolved_request_ids


def test_marker_write_failure_does_not_suppress_stop_unconfirmed_and_is_retryable(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    original = store._write
    monkeypatch.setattr(store, "_write", lambda _resources: (_ for _ in ()).throw(OSError("disk unavailable")))
    with pytest.raises(OSError, match="disk unavailable"):
        subscriber.report_shutdown_result({"arm": "stop reply missing"})
    status = subscriber.request_status_publisher.messages[-1]
    assert status.state == CommandStatus.STATE_STOP_UNCONFIRMED and not status.execution_idle
    monkeypatch.setattr(store, "_write", original)
    subscriber.report_shutdown_result({"arm": "stop reply missing"})
    assert store.load() == {"bt_runner/arm": "stop reply missing"}


@pytest.mark.parametrize("unresolved", [{}, {"arm": "terminal result missing"}])
def test_shutdown_stop_requests_join_one_coordinator_before_and_after_final_report(tmp_path, unresolved):
    calls = []
    resources = SimpleNamespace(latch_software_stop=lambda: calls.append("latch"),
                                request_software_stop=lambda: calls.append("stop"))
    subscriber = owner(StopUnconfirmedStore(root=tmp_path), resources)
    original = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(original, subscriber))
    subscriber.close_admission()

    joining = stop_request(original.request_id)
    subscriber.append_request_to_buffer(wire(joining, subscriber))
    waiting = query(subscriber, joining.request_id).status
    assert waiting.state == CommandStatus.STATE_STOP_REQUESTED
    assert not waiting.execution_idle
    assert not subscriber.pending_msgs and not subscriber.blackboard.command_buffer
    assert calls == []

    subscriber.report_shutdown_result(unresolved)
    expected = (CommandStatus.STATE_STOP_UNCONFIRMED if unresolved else
                CommandStatus.STATE_STOP_CONFIRMED)
    for request in (original, joining):
        status = query(subscriber, request.request_id).status
        assert status.state == expected
        assert status.execution_idle is (not unresolved)

    late = stop_request(original.request_id)
    subscriber.append_request_to_buffer(wire(late, subscriber))
    subscriber.append_request_to_buffer(wire(late, subscriber))
    status = query(subscriber, late.request_id).status
    assert status.state == expected and status.execution_idle is (not unresolved)
    assert not query(subscriber).accepting_commands
    assert calls == []
