"""Prior shutdown diagnostics do not become work in a fresh command session."""

from types import SimpleNamespace
import json

import pytest
from fault_detector_msgs.msg import CommandStatus
from fault_detector_msgs.srv import GetCommandStatus

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from test_bt_startup import ConsumerNode
from test_command_owner_reconciliation import wire, stop_publisher, stop_request
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


def test_restart_archives_each_owners_diagnostics_without_replay_or_stop(tmp_path):
    bt_store = StopUnconfirmedStore("bt_runner", tmp_path)
    api_store = StopUnconfirmedStore("application_api", tmp_path)
    bt_store.record({"arm": "terminal result missing", "base": "fresh velocity missing"})
    api_store.record({"acquisition": "cancellation result missing"})
    latches = []
    restarted = owner(bt_store, SimpleNamespace(latch_software_stop=lambda: latches.append(True)))
    discovery = query(restarted)
    assert latches == []
    assert discovery.accepting_commands
    assert not discovery.status.software_stop_latched
    assert not discovery.status.software_stop_confirmed
    assert not discovery.unresolved_request_ids
    assert not restarted.pending_msgs and not restarted.blackboard.command_buffer
    archive = next((tmp_path / "history").glob("*/bt_runner.json"))
    assert json.loads(archive.read_text())["unresolved"] == {
        "arm": "terminal result missing", "base": "fresh velocity missing",
    }
    assert api_store.path.exists()  # Peer history is archived by its own startup.
    assert api_store.begin_session()
    assert not bt_store.load()
    assert next((tmp_path / "history").glob("*/application_api.json")).exists()
    request = semantic_request(CommandID.READY_ARM, "operator")
    restarted.append_request_to_buffer(wire(request, restarted))
    assert query(restarted, request.request_id).status.state == CommandStatus.STATE_RUNNING


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
    assert waiting.state == CommandStatus.STATE_UNKNOWN and not waiting.execution_idle
    assert not waiting.software_stop_latched and not waiting.software_stop_confirmed
    rejected = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(rejected, subscriber))
    assert query(subscriber, rejected.request_id).status.state == CommandStatus.STATE_FAILED
    subscriber.report_shutdown_result({"arm": "late goal acceptance remains unresolved"})
    missing = query(subscriber, request.request_id).status
    assert missing.state == CommandStatus.STATE_UNKNOWN and not missing.execution_idle
    assert not missing.software_stop_latched and not missing.software_stop_confirmed
    subscriber.close_admission()
    assert query(subscriber, request.request_id).status.state == CommandStatus.STATE_UNKNOWN
    assert store.load() == {"bt_runner/arm": "late goal acceptance remains unresolved"}


def test_local_release_preserves_closed_admission_without_claiming_physical_stop(tmp_path):
    subscriber = owner(StopUnconfirmedStore(root=tmp_path))
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    subscriber.close_admission()
    subscriber.report_shutdown_result({})
    status = query(subscriber, request.request_id).status
    assert status.state == CommandStatus.STATE_CANCELLED and status.execution_idle
    assert not status.software_stop_latched and not status.software_stop_confirmed
    assert not query(subscriber).accepting_commands


@pytest.mark.parametrize("phase", ["pending", "confirmed", "confirmed_then_pending"])
def test_shutdown_preserves_only_existing_explicit_stop_confirmation(tmp_path, phase):
    subscriber = owner(StopUnconfirmedStore(root=tmp_path))
    stop = stop_request()
    subscriber.append_request_to_buffer(wire(stop, subscriber))
    if phase != "pending":
        stop_publisher(subscriber, stop).update()
    if phase == "confirmed_then_pending":
        stop = stop_request()
        subscriber.append_request_to_buffer(wire(stop, subscriber))

    subscriber.close_admission()
    subscriber.report_shutdown_result({})
    status = query(subscriber, stop.request_id).status
    assert status.software_stop_latched
    assert status.software_stop_confirmed is (phase == "confirmed")
    assert status.state == (CommandStatus.STATE_STOP_CONFIRMED if phase == "confirmed"
                            else CommandStatus.STATE_CANCELLED)
    assert not query(subscriber).accepting_commands


def test_corrupt_diagnostic_is_archived_but_corrupt_process_ownership_blocks(tmp_path):
    bt_store = StopUnconfirmedStore("bt_runner", tmp_path)
    api_store = StopUnconfirmedStore("application_api", tmp_path)
    bt_store.record({"arm": "result missing", "base": "telemetry missing"})
    api_store.record({"setup": "cancel pending"})
    bt_store.clear_confirmed(["arm"])
    assert bt_store.load() == {"bt_runner/base": "telemetry missing",
                               "application_api/setup": "cancel pending"}
    bt_store.path.write_text("not valid json", encoding="utf-8")
    restarted = owner(bt_store)
    assert query(restarted).accepting_commands
    assert next((tmp_path / "history").glob("*/bt_runner.json")).read_text() == "not valid json"
    bt_store.session_path.write_text("not valid json", encoding="utf-8")
    blocked = owner(StopUnconfirmedStore("bt_runner", tmp_path))
    assert not query(blocked).accepting_commands
    assert "shutdown:bt_runner/active_session" in query(blocked).unresolved_request_ids


def test_marker_write_failure_does_not_suppress_unresolved_status_and_is_retryable(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    original = store._write
    monkeypatch.setattr(store, "_write", lambda _resources: (_ for _ in ()).throw(OSError("disk unavailable")))
    with pytest.raises(OSError, match="disk unavailable"):
        subscriber.report_shutdown_result({"arm": "stop reply missing"})
    status = subscriber.request_status_publisher.messages[-1]
    assert status.state == CommandStatus.STATE_UNKNOWN and not status.execution_idle
    assert not status.software_stop_confirmed
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
    assert waiting.state == CommandStatus.STATE_UNKNOWN
    assert not waiting.execution_idle
    assert not waiting.software_stop_latched and not waiting.software_stop_confirmed
    assert not subscriber.pending_msgs and not subscriber.blackboard.command_buffer
    assert calls == []

    subscriber.report_shutdown_result(unresolved)
    expected = CommandStatus.STATE_UNKNOWN if unresolved else CommandStatus.STATE_CANCELLED
    for request in (original, joining):
        status = query(subscriber, request.request_id).status
        assert status.state == expected
        assert status.execution_idle is (not unresolved)
        assert not status.software_stop_latched and not status.software_stop_confirmed

    late = stop_request(original.request_id)
    subscriber.append_request_to_buffer(wire(late, subscriber))
    subscriber.append_request_to_buffer(wire(late, subscriber))
    status = query(subscriber, late.request_id).status
    assert status.state == expected and status.execution_idle is (not unresolved)
    assert not status.software_stop_latched and not status.software_stop_confirmed
    assert not query(subscriber).accepting_commands
    assert calls == []
