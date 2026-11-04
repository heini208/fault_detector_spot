"""Admission requires durable ownership even if timeout reporting later fails."""

from dataclasses import replace
import json
from threading import Event, Thread
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_msgs.msg import CommandStatus
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from test_bt_startup import ConsumerNode
from test_command_owner_reconciliation import stop_publisher, stop_request, wire
from test_command_request_correlation import semantic_request
from test_shutdown_reconciliation import owner, query


def test_marker_write_failure_keeps_startup_unadvertised(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    monkeypatch.setattr(store, "begin_session", lambda: (_ for _ in ()).throw(OSError("disk full")))
    subscriber = CommandSubscriber(shutdown_state_store=store, defer_subscription=True)
    node = ConsumerNode()
    subscriber.setup(node=node)
    with pytest.raises(OSError, match="disk full"):
        subscriber.activate()
    assert not node.subscriptions and not node.services
    assert not query(subscriber).accepting_commands


def test_lost_exclusive_session_ownership_cannot_admit_after_marker_disappears(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    monkeypatch.setattr(store, "begin_session", lambda: False)
    subscriber = owner(store)
    assert not query(subscriber).accepting_commands
    assert "shutdown:bt_runner/active_session" in query(subscriber).unresolved_request_ids


def test_report_write_failure_retains_preexisting_dirty_session_for_restart(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    assert store.session_path.exists()
    monkeypatch.setattr(store, "_write", lambda _value: (_ for _ in ()).throw(OSError("disk full")))
    with pytest.raises(OSError, match="disk full"):
        subscriber.report_shutdown_result({"arm": "result missing"})
    restarted = owner(StopUnconfirmedStore(root=tmp_path))
    status = query(restarted)
    assert not status.accepting_commands
    assert status.status.state == CommandStatus.STATE_STOP_UNCONFIRMED
    assert "shutdown:bt_runner/active_session" in status.unresolved_request_ids
    assert not restarted.confirm_shutdown_session()
    assert store.session_path.exists()


def test_live_other_owner_is_allowed_and_dead_owner_starts_fresh(tmp_path):
    bt = StopUnconfirmedStore("bt_runner", tmp_path)
    api = StopUnconfirmedStore("application_api", tmp_path)
    assert bt.begin_session()
    assert not bt.load() and not api.load()
    bt.set_session_work(True)
    marker = json.loads(bt.session_path.read_text(encoding="utf-8"))
    marker["process"]["boot"] = "12345678-1234-4234-8234-123456789012"
    bt.session_path.write_text(json.dumps(marker), encoding="utf-8")
    assert "bt_runner/active_session" in api.load()
    duplicate = StopUnconfirmedStore("bt_runner", tmp_path)
    assert duplicate.begin_session()
    assert not duplicate.load()
    assert json.loads(next((tmp_path / "history").glob("*/bt_runner.active")).read_text()) == marker


def test_only_the_session_owner_can_clear_its_marker_and_prior_resources_are_archived(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    store.record({"arm": "prior result missing"})
    assert store.begin_session()
    other = StopUnconfirmedStore(root=tmp_path)
    assert not other.begin_session()
    assert other.confirm_session()  # It owns nothing and cannot clear this file.
    assert store.session_path.exists()
    assert store.confirm_session()
    assert store.confirm_session()
    assert store.load() == {}
    archive = next((tmp_path / "history").glob("*/bt_runner.json"))
    assert json.loads(archive.read_text())["unresolved"] == {"arm": "prior result missing"}
    assert store.begin_session()
    store.clear_confirmed(["active_session"])
    assert not store.session_path.exists()


def test_late_foreign_stop_persists_after_successful_shutdown_report(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    subscriber.close_admission()
    subscriber.report_shutdown_result({})
    target = "41111111-1111-4111-8111-111111111111"
    stop = replace(stop_request(target), stop_execution_instance_ids=("prior-owner",))
    subscriber.append_request_to_buffer(wire(stop, subscriber))
    status = query(subscriber, stop.request_id).status
    assert status.state == CommandStatus.STATE_UNKNOWN and not status.execution_idle
    assert not status.software_stop_latched and not status.software_stop_confirmed
    assert "bt_runner/prior ownership" in store.load()
    assert not subscriber.confirm_shutdown_session()
    assert store.session_path.exists()


def test_dirty_session_is_cleared_only_after_failed_cleanup_succeeds_on_retry(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    monkeypatch.setattr(runner, "helper_initializer", None)
    monkeypatch.setattr(runner.rclpy, "try_shutdown", lambda: None)
    tree = SimpleNamespace(root=SimpleNamespace(iterate=lambda: (subscriber,)),
                           timer=Mock(), shutdown=Mock())
    node = SimpleNamespace(has_parameter=lambda _name: True,
                           get_parameter=lambda _name: SimpleNamespace(value=0.1),
                           get_logger=lambda: Mock(),
                           destroy_node=Mock(side_effect=[RuntimeError("node busy"), None]))
    executor = SimpleNamespace(
        spin_once=lambda timeout_sec: Event().wait(min(timeout_sec, 0.001)),
        shutdown=lambda **_kwargs: None,
    )
    coordinator = runner.make_shutdown_coordinator(node, tree, executor)
    first = coordinator.close()
    assert first.confirmed and "ROS node" in first.cleanup_errors
    assert "confirm shutdown session" in first.cleanup_errors
    assert store.session_path.exists()
    retried = coordinator.close()
    assert retried.confirmed and not retried.cleanup_errors
    assert not store.session_path.exists()


def session_has_work(store):
    return json.loads(store.session_path.read_text(encoding="utf-8"))["unresolved_work"]


def retire_session(store):
    marker = json.loads(store.session_path.read_text(encoding="utf-8"))
    marker["process"]["boot"] = "12345678-1234-4234-8234-123456789012"
    store.session_path.write_text(json.dumps(marker), encoding="utf-8")


def terminal_status(request_id):
    return CommandStatus(
        request_id=request_id, state=CommandStatus.STATE_SUCCEEDED,
        execution_idle=True,
    )


def test_idle_bt_crash_does_not_create_shutdown_work_or_latch_on_restart(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    assert query(subscriber).accepting_commands
    assert not session_has_work(store)

    retire_session(store)
    restarted = owner(StopUnconfirmedStore(root=tmp_path))

    snapshot = query(restarted)
    assert snapshot.accepting_commands
    assert not snapshot.status.software_stop_latched
    assert snapshot.unresolved_request_ids == []
    assert not restarted.pending_msgs and not restarted.blackboard.command_buffer


@pytest.mark.parametrize("completed", [False, True])
def test_bt_restart_starts_fresh_and_archives_original_execution_state(tmp_path, completed):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    assert session_has_work(store)
    if completed:
        subscriber.status_ledger.record(terminal_status(request.request_id))
        assert not session_has_work(store)

    retire_session(store)
    restarted = owner(StopUnconfirmedStore(root=tmp_path))

    snapshot = query(restarted)
    assert snapshot.accepting_commands
    assert not snapshot.status.software_stop_latched
    assert not snapshot.unresolved_request_ids
    assert not restarted.pending_msgs and not restarted.blackboard.command_buffer
    archive = next((tmp_path / "history").glob("*/bt_runner.active"))
    assert json.loads(archive.read_text())["unresolved_work"] is (not completed)


def test_failed_durable_admission_never_buffers_or_runs_command(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.status_ledger.configure_session_work(
        Mock(side_effect=OSError("disk full")),
    )

    with pytest.raises(OSError, match="disk full"):
        subscriber.append_request_to_buffer(wire(request, subscriber))

    assert not subscriber.status_ledger.contains(request.request_id)
    assert not subscriber.pending_msgs and not subscriber.blackboard.command_buffer
    assert not session_has_work(store)


def test_rejected_dispatch_keeps_idle_session_clean(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    invalid = wire(request, subscriber)
    invalid.client_id = ""
    subscriber.append_request_to_buffer(invalid)

    assert query(subscriber).accepting_commands
    assert not session_has_work(store)
    assert not subscriber.pending_msgs and not subscriber.blackboard.command_buffer


@pytest.mark.parametrize("reset", [False, True])
def test_explicit_stop_is_local_to_the_session_and_archived_on_restart(tmp_path, reset):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    stop = stop_request()
    subscriber.append_request_to_buffer(wire(stop, subscriber))
    stop_publisher(subscriber, stop).update()
    assert query(subscriber).status.software_stop_confirmed
    assert session_has_work(store)
    if reset:
        request = replace(
            semantic_request(CommandID.RESET_SOFTWARE_STOP, "operator"),
            execution_instance_id=subscriber.status_ledger.execution_instance_id,
        )
        assert subscriber.status_ledger.receive(request)[0]
        assert subscriber.status_ledger.reset_stop(
            request.request_id, lambda clear: (clear() or True, "reset confirmed"),
        )[0]
        assert session_has_work(store)  # The reset request still owns execution.
        subscriber.status_ledger.record(terminal_status(request.request_id))
        assert not session_has_work(store)

    retire_session(store)
    restarted = owner(StopUnconfirmedStore(root=tmp_path))
    assert query(restarted).accepting_commands
    assert not query(restarted).status.software_stop_latched
    archive = next((tmp_path / "history").glob("*/bt_runner.active"))
    assert json.loads(archive.read_text())["unresolved_work"] is (not reset)


def test_stop_ingress_prevents_concurrent_terminal_feedback_clearing_session(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    request = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(request, subscriber))
    persisted = Event()
    original = subscriber.status_ledger.retain_stop_intent

    def retain():
        original()
        persisted.set()

    subscriber.status_ledger.retain_stop_intent = retain
    stop = stop_request(request.request_id)
    with subscriber.status_ledger._lock:
        worker = Thread(target=subscriber.append_request_to_buffer, args=(wire(stop, subscriber),))
        worker.start()
        assert persisted.wait(2), "Stop persistence waited for normal command feedback"
        subscriber.status_ledger.record(terminal_status(request.request_id))
        assert session_has_work(store)
        assert not query(subscriber).accepting_commands
    worker.join(2)
    assert not worker.is_alive()
    assert session_has_work(store)


def test_fully_confirmed_shutdown_clears_stop_session(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    stop = stop_request()
    subscriber.append_request_to_buffer(wire(stop, subscriber))
    stop_publisher(subscriber, stop).update()
    assert session_has_work(store)

    subscriber.close_admission()
    subscriber.report_shutdown_result({})
    assert subscriber.confirm_shutdown_session()

    assert not store.session_path.exists()
    restarted = owner(StopUnconfirmedStore(root=tmp_path))
    assert query(restarted).accepting_commands


def test_failed_terminal_fsync_cannot_leave_next_execution_durably_idle(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    subscriber = owner(store)
    writes = []

    def persist(unresolved):
        store.set_session_work(unresolved)
        writes.append(unresolved)
        if not unresolved:
            raise OSError("directory fsync failed after replace")

    subscriber.status_ledger.configure_session_work(persist)
    first = semantic_request(CommandID.READY_ARM, "operator")
    subscriber.append_request_to_buffer(wire(first, subscriber))
    with pytest.raises(OSError, match="fsync failed"):
        subscriber.status_ledger.record(terminal_status(first.request_id))
    assert not session_has_work(store)

    second = semantic_request(CommandID.STOW_ARM, "operator")
    subscriber.append_request_to_buffer(wire(second, subscriber))

    assert writes == [True, False, True]
    assert session_has_work(store)
    retire_session(store)
    assert query(owner(StopUnconfirmedStore(root=tmp_path))).accepting_commands
    archive = next((tmp_path / "history").glob("*/bt_runner.active"))
    assert json.loads(archive.read_text())["unresolved_work"] is True
