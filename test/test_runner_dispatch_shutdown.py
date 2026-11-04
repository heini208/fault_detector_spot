"""Offline dispatch fencing covers queued and already-running tree ticks."""

from concurrent.futures import Future
from dataclasses import replace
from threading import Event, Thread
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources, _StopAwareActionClient,
)
from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
from test_command_owner_reconciliation import consumer, stop_request, wire


def ticking_tree(tick):
    callbacks = []
    timer = Mock()
    node = SimpleNamespace(create_timer=lambda _period, callback, **_kwargs:
                           (callbacks.append(callback), timer)[1])
    tree = SimpleNamespace(node=node, tick=tick, root=SimpleNamespace(iterate=lambda: ()),
                           shutdown=Mock())
    runner.start_tree_ticking(tree)
    return tree, callbacks[0]


@pytest.mark.parametrize("finish_tick", [True, False])
def test_shutdown_drains_active_dispatch_or_reports_it_within_deadline(monkeypatch, finish_tick):
    started, release = Event(), Event()

    def tick():
        started.set()
        release.wait()

    tree, timer_callback = ticking_tree(tick)
    worker = Thread(target=timer_callback, daemon=True)
    worker.start()
    assert started.wait(1.0)
    events = []
    monkeypatch.setattr(runner, "helper_initializer", None)
    monkeypatch.setattr(runner.rclpy, "try_shutdown", lambda: events.append("ROS stopped"))
    node = SimpleNamespace(
        has_parameter=lambda _name: True,
        get_parameter=lambda _name: SimpleNamespace(value=0.1),
        get_logger=lambda: Mock(),
        destroy_node=lambda: events.append("node destroyed"),
    )

    def callbacks(timeout_sec):
        if finish_tick:
            release.set()
        Event().wait(min(timeout_sec, 0.001))

    executor = SimpleNamespace(spin_once=callbacks,
                               shutdown=lambda **_kwargs: events.append("executor stopped"))
    try:
        shutdown = runner.make_shutdown_coordinator(node, tree, executor)
        # Even a callback queued before timer.cancel is fenced immediately.
        timer_callback()
        report = shutdown.close()
        assert report.confirmed is finish_tick
        assert ("tree dispatch" in report.unresolved) is (not finish_tick)
        assert not report.cleanup_errors
        assert events == ["executor stopped", "node destroyed", "ROS stopped"]
        assert shutdown.close() == report
    finally:
        release.set()
        worker.join(timeout=1.0)
    assert not worker.is_alive()


def test_dispatch_exception_releases_activity_and_pause_blocks_late_callbacks():
    tick = Mock(side_effect=ValueError("tick failed"))
    tree, callback = ticking_tree(tick)
    with pytest.raises(ValueError, match="tick failed"):
        callback()
    assert tree._shutdown_dispatch.status()[0]
    tree._shutdown_dispatch.pause()
    callback()
    tick.assert_called_once()


def test_stop_preparation_creates_all_physical_owners_before_dispatch():
    resources = RobotCommandResources()
    calls = []
    resources.get_arm_movement_executor = lambda _node: calls.append("arm")
    resources.get_base_movement_executor = lambda _node: calls.append("base")
    resources.get_waypoint_navigation_executor = lambda _node: calls.append("nav2")
    node = SimpleNamespace(create_publisher=lambda *_args: object(),
                           create_subscription=lambda *_args, **_kwargs: object())
    resources.prepare_stop(node)
    assert calls == ["arm", "base", "nav2"]


def test_shutdown_freezes_late_resource_lookup_and_cannot_be_reopened_by_old_reset():
    resources = RobotCommandResources()
    node = object()
    resources._bind_node(node)
    client = Mock()
    client.send_goal_async.return_value = Future()
    wrapped = _StopAwareActionClient(client, lambda: resources.stop_latched, lambda: None)
    resources.close_admission()
    # An old reset may have already passed its guard before shutdown fenced it.
    resources._software_stop.clear()
    assert resources.stop_latched
    assert not wrapped.send_goal_async(object()).result().accepted
    client.send_goal_async.assert_not_called()
    for lookup in (resources.get_arm_movement_executor, resources.get_base_movement_executor,
                   resources.get_waypoint_navigation_executor):
        with pytest.raises(RuntimeError, match="resource admission"):
            lookup(node)


def test_runner_closes_resource_admission_before_owner_and_cleanup_snapshots(monkeypatch):
    resources = RobotCommandResources()
    events = []

    def owners():
        assert resources.stop_latched and resources._closing
        events.append("owners")
        return {}

    def cleanup():
        assert resources.stop_latched and resources._closing
        events.append("cleanup")
        return {}

    resources.shutdown_owners = owners
    resources.cleanup_operations = cleanup
    helper = SimpleNamespace(robot_command_resources=resources, cleanup_operations=lambda: {})
    monkeypatch.setattr(runner, "helper_initializer", helper)
    node = SimpleNamespace(has_parameter=lambda _name: True,
                           get_parameter=lambda _name: SimpleNamespace(value=1.0),
                           destroy_node=Mock())
    executor = SimpleNamespace(spin_once=Mock(), shutdown=Mock())
    runner.make_shutdown_coordinator(node, None, executor)
    assert events == ["owners", "owners", "cleanup"]


def test_foreign_request_ownership_remains_unconfirmed_when_local_stops_succeed(monkeypatch):
    subscriber = consumer()
    missing_request_id = "41111111-1111-4111-8111-111111111111"
    stop = replace(stop_request(missing_request_id),
                   stop_execution_instance_ids=("previous-execution-instance",))
    subscriber.append_request_to_buffer(wire(stop, subscriber))
    resources = RobotCommandResources()
    resources.begin_shutdown = lambda: None
    resources.poll_shutdown = lambda: (True, "Local sources stopped")
    helper = SimpleNamespace(robot_command_resources=resources, cleanup_operations=lambda: {})
    monkeypatch.setattr(runner, "helper_initializer", helper)
    monkeypatch.setattr(runner.rclpy, "try_shutdown", lambda: None)
    tree = SimpleNamespace(root=SimpleNamespace(iterate=lambda: (subscriber,)),
                           timer=Mock(), shutdown=Mock())
    node = SimpleNamespace(has_parameter=lambda _name: True,
                           get_parameter=lambda _name: SimpleNamespace(value=0.1),
                           get_logger=lambda: Mock(), destroy_node=Mock())
    executor = SimpleNamespace(
        spin_once=lambda timeout_sec: Event().wait(min(timeout_sec, 0.001)),
        shutdown=lambda **_kwargs: None,
    )
    report = runner.make_shutdown_coordinator(node, tree, executor).close()
    assert not report.confirmed and not report.cleanup_errors
    assert "prior ownership/0" in report.unresolved
    assert missing_request_id in report.unresolved["prior ownership/0"]
    assert "bt_runner/prior ownership/0" in StopUnconfirmedStore().load()
    restarted = consumer()
    # Its restored marker is represented once by prior-shutdown confirmation,
    # rather than becoming another nested prior-ownership marker.
    assert restarted.shutdown_ownership_status()[0]
    assert restarted.status_ledger.stop_latched
