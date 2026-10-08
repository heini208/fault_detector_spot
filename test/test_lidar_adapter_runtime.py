"""Shared lidar lifecycle with graph discovery and process execution mocked."""

from concurrent.futures import Future
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.clock import ClockType

from fault_detector_spot.sensing import lidar_adapter_runtime as module
from fault_detector_spot.shared.ros import runtime_manager as runtime_module


class Blackboard(SimpleNamespace):
    def register_key(self, *_args, **_kwargs):
        pass

    def exists(self, name):
        return hasattr(self, name)


@pytest.fixture
def runtime(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = Mock()
    node.get_parameter.return_value = SimpleNamespace(value=True)
    node.get_publishers_info_by_topic.return_value = []
    client = node.create_client.return_value
    client.service_is_ready.return_value = False
    future = Future()
    future.set_result(SimpleNamespace(success=True, message="stopping"))
    client.call_async.return_value = future
    mapping = Mock()
    mapping.is_running.return_value = False
    mapping.nav2_runtime.is_running.return_value = False
    mapping.has_pending_operation.return_value = False
    collision = Mock()
    policy = SimpleNamespace(enabled=False)
    collision.state.return_value = policy
    processes, launches, stops = [], [], []

    def launch(args, **kwargs):
        assert kwargs == {"start_new_session": True}
        process = Mock(pid=1000 + len(processes), alive=True)
        process.poll.return_value = None
        processes.append(process)
        launches.append(args)
        return process

    def stop(process, **kwargs):
        if process is not None:
            process.alive = False
            stops.append(process)
        return True

    monkeypatch.setattr(runtime_module.subprocess, "Popen", launch)
    monkeypatch.setattr(runtime_module, "is_process_group_running", lambda p: p is not None and p.alive)
    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    manager = module.LidarAdapterRuntime(node, Blackboard(), mapping, collision)
    yield SimpleNamespace(
        manager=manager, node=node, mapping=mapping, policy=policy,
        launches=launches, stops=stops, now=now, client=client,
    )
    manager.close()


def publisher(name="lidar_frame_adapter"):
    return SimpleNamespace(node_name=name, node_namespace="/")


def test_idle_startup_uses_no_adapter_and_lifecycle_clock_is_steady(runtime):
    r = runtime
    r.now[0] += 3
    r.manager._reconcile()
    assert r.launches == []
    assert r.node.create_timer.call_args.kwargs["clock"].clock_type == ClockType.STEADY_TIME
    r.client.call_async.assert_not_called()


@pytest.mark.parametrize("consumer", ["mapping", "localization", "collision"])
def test_each_consumer_starts_one_adapter_and_passes_sim_clock(runtime, consumer):
    r = runtime
    if consumer == "mapping":
        r.mapping.is_running.return_value = True
    elif consumer == "localization":
        r.mapping.nav2_runtime.is_running.return_value = True
    else:
        r.policy.enabled = True
    r.manager._reconcile()
    assert not r.launches  # Initial DDS discovery grace.
    r.now[0] += 3
    r.manager._reconcile()
    r.manager._reconcile()
    assert len(r.launches) == 1
    assert r.launches[0] == [
        "ros2", "launch", "fault_detector_spot", "lidar_frame_adapter_launch.py",
        "use_sim_time:=true",
    ]


@pytest.mark.parametrize("first_disabled", ["mapping", "collision"])
def test_only_last_consumer_stops_shared_adapter(runtime, first_disabled):
    r = runtime
    r.mapping.is_running.return_value = True
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    process = r.manager.process
    if first_disabled == "mapping":
        r.mapping.is_running.return_value = False
    else:
        r.policy.enabled = False
    r.manager._reconcile()
    assert r.manager.process is process and process.alive
    assert r.stops == []
    r.mapping.is_running.return_value = False
    r.policy.enabled = False
    r.manager._reconcile()
    assert r.stops == [process]
    assert r.manager.process is None


def test_map_switch_and_save_keep_source_until_operation_finishes(runtime):
    r = runtime
    r.mapping.is_running.return_value = True
    r.now[0] += 3
    r.manager._reconcile()
    r.mapping.is_running.return_value = False
    r.mapping.has_pending_operation.return_value = True
    r.manager._reconcile()
    assert not r.stops
    r.mapping.is_running.return_value = True
    r.mapping.has_pending_operation.return_value = False
    r.manager._reconcile()
    assert len(r.launches) == 1 and not r.stops


def test_existing_adapter_reused_even_without_fresh_scans_then_stopped_cooperatively(runtime):
    r = runtime
    r.node.get_publishers_info_by_topic.return_value = [publisher()]
    r.client.service_is_ready.return_value = True
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    assert not r.launches
    r.client.call_async.assert_not_called()
    r.policy.enabled = False
    r.manager._reconcile()
    r.client.call_async.assert_called_once()
    assert not r.stops  # Never signal a process group we did not create.
    r.manager._reconcile()
    assert r.client.call_async.call_count == 1  # Wait for discovery to catch up.


def test_adapter_starting_service_before_cloud_publisher_prevents_duplicate(runtime):
    r = runtime
    r.policy.enabled = True
    r.client.service_is_ready.return_value = True
    r.now[0] += 3
    r.manager._reconcile()
    assert not r.launches


def test_old_adapter_reused_but_not_killed_without_shutdown_service(runtime):
    r = runtime
    r.node.get_publishers_info_by_topic.return_value = [publisher()]
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    assert not r.launches
    r.policy.enabled = False
    r.manager._reconcile()
    r.manager._reconcile()
    r.node.get_logger().warning.assert_called_once()
    assert "Stop the old manual adapter once" in r.node.get_logger().warning.call_args.args[0]
    r.client.call_async.assert_not_called()
    assert not r.stops


@pytest.mark.parametrize("failure", ["timeout", "refused", "exception"])
def test_external_shutdown_failure_is_reported_without_duplicate_or_process_signal(runtime, monkeypatch, failure):
    r = runtime
    r.node.get_publishers_info_by_topic.return_value = [publisher()]
    r.client.service_is_ready.return_value = True
    future = Future()
    if failure == "refused":
        future.set_result(SimpleNamespace(success=False, message="busy"))
    elif failure == "exception":
        future.set_exception(RuntimeError("transport failed"))
    else:
        event = Mock()
        event.wait.return_value = False
        monkeypatch.setattr(module, "Event", lambda: event)
    r.client.call_async.return_value = future
    r.manager._reconcile()
    r.node.get_logger().warning.assert_called_once()
    assert not r.launches and not r.stops
    if failure == "timeout":
        event.wait.assert_called_once_with(2.0)
        r.client.remove_pending_request.assert_called_once_with(future)
    r.manager._reconcile()
    r.client.call_async.assert_called_once()  # No tight retry loop.


@pytest.mark.parametrize("publishers", [[publisher("rosbag2_player")], [publisher(), publisher()]])
def test_other_or_ambiguous_publishers_are_not_stopped(runtime, publishers):
    r = runtime
    r.node.get_publishers_info_by_topic.return_value = publishers
    r.client.service_is_ready.return_value = True
    r.manager._reconcile()
    r.client.call_async.assert_not_called()
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    assert not r.launches and not r.stops


def test_failed_launch_retries_with_backoff_without_changing_collision_preference(runtime, monkeypatch):
    r = runtime
    r.policy.enabled = True
    launch = Mock(side_effect=OSError("missing ros2"))
    monkeypatch.setattr(runtime_module.subprocess, "Popen", launch)
    r.now[0] += 3
    r.manager._reconcile()
    r.manager._reconcile()
    assert launch.call_count == 1
    assert r.manager.process is None and r.policy.enabled
    r.now[0] += 5
    r.manager._reconcile()
    assert launch.call_count == 2


def test_adapter_exit_reaps_old_group_before_replacement(runtime):
    r = runtime
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    old = r.manager.process
    old.poll.return_value = 1
    r.now[0] += 5
    r.manager._reconcile()
    assert r.stops == [old]
    assert len(r.launches) == 2
    assert r.manager.process is not old


def test_slow_stop_does_not_block_timer_and_latest_toggle_wins(runtime, monkeypatch):
    r = runtime
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    entered, release = Event(), Event()
    original = runtime_module.terminate_process_group

    def stop(process, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(process, **kwargs)

    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    r.policy.enabled = False
    try:
        r.manager._schedule_reconcile()
        assert entered.wait(1)
        r.policy.enabled = True
        r.manager._schedule_reconcile()
        assert not r.manager._runtime_future.done()
        assert len(r.launches) == 1
    finally:
        release.set()
    r.manager._runtime_future.result(timeout=1)
    r.now[0] += 3
    r.manager._schedule_reconcile()
    r.manager._runtime_future.result(timeout=1)
    assert len(r.launches) == 2 and len(r.stops) == 1


def test_stop_failure_retains_owned_group_for_retry(runtime, monkeypatch):
    r = runtime
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    process = r.manager.process
    r.policy.enabled = False
    original = runtime_module.terminate_process_group
    monkeypatch.setattr(runtime_module, "terminate_process_group", lambda *args, **kwargs: False)
    r.manager._reconcile()
    assert r.manager.process is process
    monkeypatch.setattr(runtime_module, "terminate_process_group", original)
    r.manager._reconcile()
    assert r.manager.process is None


def test_close_stops_owned_group_and_prevents_late_timer_launch(runtime):
    r = runtime
    r.policy.enabled = True
    r.now[0] += 3
    r.manager._reconcile()
    assert r.manager.close()
    assert r.manager.close()
    r.manager._schedule_reconcile()
    r.manager._reconcile()
    assert len(r.launches) == len(r.stops) == 1
    r.node.destroy_timer.assert_called_once()
    r.node.destroy_client.assert_called_once()


def test_disabled_during_start_is_reconciled_without_leaking_adapter(runtime, monkeypatch):
    r = runtime
    r.policy.enabled = True
    r.now[0] += 3
    entered, release = Event(), Event()
    original = runtime_module.subprocess.Popen

    def launch(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(*args, **kwargs)

    monkeypatch.setattr(runtime_module.subprocess, "Popen", launch)
    try:
        r.manager._schedule_reconcile()
        assert entered.wait(1)
        r.policy.enabled = False
    finally:
        release.set()
    r.manager._runtime_future.result(timeout=1)
    r.manager._schedule_reconcile()
    r.manager._runtime_future.result(timeout=1)
    assert len(r.launches) == len(r.stops) == 1
    assert r.manager.process is None


def test_helper_shares_consumers_and_routes_only_managed_mapping_to_corrected_cloud(monkeypatch, tmp_path):
    from fault_detector_spot.application.behaviour_tree.behaviours import helper_initializer as helper_module

    for name in ("RobotCommandResources", "TagStateSource", "RtabmapRuntimeManager", "LidarAdapterRuntime"):
        monkeypatch.setattr(helper_module, name, Mock())
    helper_module.LidarAdapterRuntime.OUTPUT_TOPIC = module.LidarAdapterRuntime.OUTPUT_TOPIC
    node = Mock()
    node.get_parameter.side_effect = lambda name: SimpleNamespace(
        value=str(tmp_path) if name == "navigation.map_root" else 1.5,
    )
    helper = helper_module.HelperInitializer("test", node)
    assert helper.setup(timeout=1)
    assert helper_module.RtabmapRuntimeManager.call_args.kwargs["raw_lidar_topic"] == "/velodyne/points_sensor"
    helper_module.LidarAdapterRuntime.assert_called_once_with(
        node, helper.bb_client, helper.rtabmap_runtime,
        helper.robot_command_resources.get_arm_collision_control.return_value,
    )
