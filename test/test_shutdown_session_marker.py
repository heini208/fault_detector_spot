"""Admission requires durable ownership even if timeout reporting later fails."""

from dataclasses import replace
import json
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_msgs.msg import CommandStatus
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from test_bt_startup import ConsumerNode
from test_command_owner_reconciliation import stop_request, wire
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


def test_live_other_owner_is_allowed_but_dead_or_rebooted_session_is_unresolved(tmp_path):
    bt = StopUnconfirmedStore("bt_runner", tmp_path)
    api = StopUnconfirmedStore("application_api", tmp_path)
    assert bt.begin_session()
    assert not bt.load() and not api.load()
    marker = json.loads(bt.session_path.read_text(encoding="utf-8"))
    marker["process"]["boot"] = "another-boot"
    bt.session_path.write_text(json.dumps(marker), encoding="utf-8")
    assert "bt_runner/active_session" in api.load()
    duplicate = StopUnconfirmedStore("bt_runner", tmp_path)
    assert not duplicate.begin_session()
    assert "bt_runner/active_session" in duplicate.load()


def test_only_the_session_owner_can_clear_its_marker_and_prior_resources_remain(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    store.record({"arm": "prior result missing"})
    assert store.begin_session()
    other = StopUnconfirmedStore(root=tmp_path)
    assert not other.begin_session()
    assert other.confirm_session()  # It owns nothing and cannot clear this file.
    assert store.session_path.exists()
    assert store.confirm_session()
    assert store.confirm_session()
    assert store.load() == {"bt_runner/arm": "prior result missing"}
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
    assert query(subscriber, stop.request_id).status.state == CommandStatus.STATE_STOP_UNCONFIRMED
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
