"""Runtime lifecycle tests with ROS launches and process signals mocked out."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import py_trees

from fault_detector_spot.mapping.behaviours.enable_localization import EnableLocalization
from fault_detector_spot.shared.ros import runtime_manager as runtime_module
from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager
from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager


class Blackboard(SimpleNamespace):
    def register_key(self, *_args, **_kwargs):
        pass

    def exists(self, name):
        return hasattr(self, name)


@pytest.fixture
def runtime(monkeypatch, tmp_path):
    processes = []
    launches = []
    stops = []
    node = Mock()
    node.get_parameter.return_value = SimpleNamespace(value=True)

    def launch(args, **kwargs):
        assert kwargs == {"start_new_session": True}
        process = SimpleNamespace(pid=100 + len(processes), alive=True)
        processes.append(process)
        launches.append(args)
        return process

    def stop(process, **kwargs):
        stops.append((process, kwargs))
        process.alive = False
        return True

    monkeypatch.setattr(runtime_module.subprocess, "Popen", launch)
    monkeypatch.setattr(runtime_module, "is_process_group_running", lambda p: p is not None and p.alive)
    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    manager = RtabmapRuntimeManager(node, Blackboard(), maps_dir=tmp_path, nav2_params_file=None)
    manager._call_service = Mock(return_value=True)
    yield manager, launches, stops
    for process in processes:
        process.alive = False
    manager.close()


def test_shared_parent_and_launch_configuration(runtime):
    manager, launches, stops = runtime
    assert isinstance(manager, RuntimeManager)
    assert isinstance(manager.nav2_runtime, RuntimeManager)
    process = manager.start_mapping("plant", rviz=False)
    assert launches[0][:4] == ["ros2", "launch", "fault_detector_spot", "lidar_rtab_mapping_launch.py"]
    assert "use_sim_time:=true" in launches[0]
    assert "rviz:=false" in launches[0]
    assert "extend_map:=true" in launches[0]
    assert manager.process is process
    assert manager.is_mapping_running()


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_managed_mapping_and_localization_forward_corrected_lidar(runtime, tmp_path, mode):
    manager, launches, stops = runtime
    manager.raw_lidar_topic = "/velodyne/points_sensor"
    (tmp_path / "plant.db").touch()
    start = manager.start_mapping if mode == "mapping" else manager.start_localization
    start("plant", rviz=False)
    assert "raw_lidar_topic:=/velodyne/points_sensor" in launches[0]


def test_nav2_duplicate_start_does_not_orphan_process(runtime):
    manager, launches, stops = runtime
    nav2 = manager.nav2_runtime
    first = nav2.start(extra_args=["use_sim_time:=false"])
    assert nav2.start() is first
    assert len(launches) == 1
    assert launches[0].count("use_sim_time:=false") == 1
    assert "use_sim_time:=true" not in launches[0]


def test_stop_failure_retains_ownership_and_close_can_be_retried(runtime, monkeypatch):
    manager, launches, stops = runtime
    nav2 = manager.nav2_runtime
    process = nav2.start()
    original = runtime_module.terminate_process_group
    monkeypatch.setattr(runtime_module, "terminate_process_group", lambda *a, **kw: False)
    assert nav2.stop() is False
    assert nav2.process is process
    with pytest.raises(RuntimeError, match="did not terminate"):
        nav2.close()
    assert not nav2._closed
    with pytest.raises(RuntimeError, match="shutting down"):
        nav2.start()
    monkeypatch.setattr(runtime_module, "terminate_process_group", original)
    assert nav2.close()
    assert nav2.close()
    assert nav2.process is None


def test_rtab_failure_still_stops_nav2(runtime, monkeypatch):
    manager, launches, stops = runtime
    rtab = manager.start_mapping("plant")
    nav2 = manager.nav2_runtime.start()
    original = runtime_module.terminate_process_group
    def stop(process, **kwargs):
        return False if process is rtab else original(process, **kwargs)
    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    assert manager.stop(save=False) is False
    assert manager.process is rtab
    assert manager.get_running_mode() == manager.MODE_MAPPING
    assert manager.nav2_runtime.process is None
    assert not nav2.alive
    monkeypatch.setattr(runtime_module, "terminate_process_group", original)


def test_save_and_no_save_shutdown_paths(runtime):
    manager, launches, stops = runtime
    manager.start_mapping("plant")
    assert manager.stop()
    assert [call.args[0] for call in manager._call_service.call_args_list] == [
        "/rtabmap/pause", "/rtabmap/save_db", "/rtabmap/publish_map",
    ]
    assert manager.get_running_mode() == manager.MODE_NONE
    manager._call_service.reset_mock()
    manager.start_mapping("plant")
    assert manager.stop(save=False)
    manager._call_service.assert_not_called()


def test_localization_launch_and_map_switch_use_one_launch_path(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    (tmp_path / "two.db").touch()
    first = manager.start_localization("one")
    assert manager.is_localization_running()
    assert "extend_map:=false" in launches[0]
    second = manager.start_localization("two")
    assert not first.alive
    assert second is not first
    assert manager.bb.active_map_name == "two"
    assert manager.is_localization_running()
    assert len(launches) == 4  # RTAB and Nav2 for each database.


@pytest.mark.parametrize("target_map", ["one", "two"])
def test_localization_replaces_mapping_even_for_the_same_map(runtime, tmp_path, target_map):
    manager, launches, stops = runtime
    (tmp_path / f"{target_map}.db").touch()
    previous = manager.start_mapping("one")

    process = manager.start_localization(target_map, rviz=False)

    assert process is not previous
    assert not previous.alive
    assert manager.bb.active_map_name == target_map
    assert manager.is_localization_running()
    assert [call.args[0] for call in manager._call_service.call_args_list] == [
        "/rtabmap/pause", "/rtabmap/save_db", "/rtabmap/publish_map",
    ]
    assert len(launches) == 3  # Old mapping, replacement localization, Nav2.
    assert f"db_path:={tmp_path / (target_map + '.db')}" in launches[1]
    assert "extend_map:=false" in launches[1]
    assert "delete_db:=false" in launches[1]
    assert "rviz:=false" in launches[1]


def test_matching_localization_reuses_running_processes(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    process = manager.start_localization("one")
    nav2 = manager.nav2_runtime.process

    assert manager.start_localization("one") is process
    assert manager.start_localization() is process
    assert manager.nav2_runtime.process is nav2
    assert len(launches) == 2
    assert stops == []
    manager._call_service.assert_not_called()


def test_matching_localization_recovers_nav2_without_restarting_map(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    process = manager.start_localization("one")
    previous_nav2 = manager.nav2_runtime.process
    previous_nav2.alive = False

    assert manager.start_localization("one") is process
    assert manager.nav2_runtime.process is not previous_nav2
    assert manager.is_localization_running()
    assert len(launches) == 3
    assert process.alive


def test_cancelled_launch_completion_cannot_satisfy_next_map_request(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    (tmp_path / "two.db").touch()
    behavior = EnableLocalization(manager)
    behavior.blackboard = manager.bb
    manager.bb.last_command = SimpleNamespace(map_name="one")
    behavior.tick_once()
    assert behavior.status == py_trees.common.Status.RUNNING
    assert manager._runtime_future.result(timeout=2)
    assert manager.bb.active_map_name == "one"

    # Cancel before the leaf consumes its completed future, then reuse the
    # command leaf for a different saved routine map.
    behavior.stop(py_trees.common.Status.INVALID)
    manager.bb.last_command = SimpleNamespace(map_name="two")
    behavior.tick_once()
    assert behavior.status == py_trees.common.Status.RUNNING
    assert manager._runtime_future.result(timeout=2)
    behavior.tick_once()

    assert behavior.status == py_trees.common.Status.SUCCESS
    assert manager.bb.active_map_name == "two"
    assert manager.is_localization_running()
    assert len(launches) == 4


@pytest.mark.parametrize("failed_runtime", ["rtab", "nav2"])
def test_localization_does_not_launch_replacement_until_both_processes_stop(
    runtime, tmp_path, monkeypatch, failed_runtime,
):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    (tmp_path / "two.db").touch()
    previous = manager.start_localization("one")
    nav2 = manager.nav2_runtime.process
    failed_process = previous if failed_runtime == "rtab" else nav2
    original = runtime_module.terminate_process_group

    def stop(process, **kwargs):
        return False if process is failed_process else original(process, **kwargs)

    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    with pytest.raises(RuntimeError, match="Could not stop runtime"):
        manager.start_localization("two")
    assert len(launches) == 2
    assert manager.bb.active_map_name == "one"
    assert failed_process.alive
    owner = manager if failed_runtime == "rtab" else manager.nav2_runtime
    assert owner.process is failed_process
    monkeypatch.setattr(runtime_module, "terminate_process_group", original)


def test_invalid_localization_map_does_not_stop_current_runtime(runtime):
    manager, launches, stops = runtime
    process = manager.start_mapping("one")
    with pytest.raises(FileNotFoundError):
        manager.start_localization("missing")
    assert process.alive
    assert manager.bb.active_map_name == "one"
    assert manager.is_mapping_running()


def test_switching_back_to_mapping_stops_nav2_without_relaunching_rtab(runtime, tmp_path):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    process = manager.start_localization("one")
    assert manager.start_mapping() is process
    assert not manager.nav2_runtime.is_running()
    assert manager.is_mapping_running()
    assert len(launches) == 2
    manager._call_service.assert_called_with("/rtabmap/set_mode_mapping")


def test_background_operation_busy_poll_and_error_propagation(runtime):
    manager, launches, stops = runtime
    entered, release = Event(), Event()
    def operation():
        entered.set()
        assert release.wait(2)
        raise ValueError("failure")
    try:
        assert manager.begin_runtime_operation("operation", operation)
        assert entered.wait(1)
        assert manager.has_pending_operation()
        assert not manager.begin_runtime_operation("other", lambda: True)
        assert manager.poll_runtime_operation("operation") is None
        with pytest.raises(RuntimeError, match="Another"):
            manager.poll_runtime_operation("other")
    finally:
        release.set()
    with pytest.raises(ValueError, match="failure"):
        manager._runtime_future.result(timeout=1)
    with pytest.raises(ValueError, match="failure"):
        manager.poll_runtime_operation("operation")
    assert manager._runtime_future is None
    assert not manager.has_pending_operation()


def test_slow_process_stop_does_not_block_operation_poll(runtime, monkeypatch):
    manager, launches, stops = runtime
    manager.start_mapping("plant")
    entered, release = Event(), Event()
    original = runtime_module.terminate_process_group
    def stop(process, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(process, **kwargs)
    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    try:
        manager.begin_runtime_operation("stop", manager.stop, save=False)
        assert entered.wait(1)
        with ThreadPoolExecutor(max_workers=1) as polling:
            future = polling.submit(manager.poll_runtime_operation, "stop")
            try:
                assert future.result(timeout=0.5) is None
            finally:
                release.set()
    finally:
        release.set()
    assert manager._runtime_future.result(timeout=1)
    assert manager.poll_runtime_operation("stop")


def test_close_rejects_new_work_and_stops_both_managers(runtime):
    manager, launches, stops = runtime
    manager.start_mapping("plant")
    manager.nav2_runtime.start()
    assert manager.close()
    assert manager.close()
    assert len(stops) == 2
    assert manager.nav2_runtime._closed
    with pytest.raises(RuntimeError, match="shutting down"):
        manager.begin_runtime_operation("late", lambda: True)
    with pytest.raises(RuntimeError, match="shutting down"):
        manager.nav2_runtime.start()


def test_launch_failure_does_not_claim_a_process(runtime, monkeypatch):
    manager, launches, stops = runtime
    monkeypatch.setattr(runtime_module.subprocess, "Popen", Mock(side_effect=OSError("launch failed")))
    with pytest.raises(OSError, match="launch failed"):
        manager.nav2_runtime.start()
    assert manager.nav2_runtime.process is None
    assert not manager.nav2_runtime.is_running()


def test_mode_switch_failure_preserves_previous_mode(runtime):
    manager, launches, stops = runtime
    manager.start_mapping("plant")
    manager._call_service.return_value = False
    with pytest.raises(RuntimeError, match="Could not switch"):
        manager.set_mode_localization()
    assert manager.get_running_mode() == manager.MODE_MAPPING
    assert not manager.nav2_runtime.is_running()


@pytest.mark.parametrize("failed_runtime", ["rtab", "nav2"])
def test_system_close_attempts_each_process_once_and_keeps_failed_owner(runtime, monkeypatch, failed_runtime):
    manager, launches, stops = runtime
    rtab = manager.start_mapping("plant")
    nav2 = manager.nav2_runtime.start()
    failed_process = rtab if failed_runtime == "rtab" else nav2
    attempts = []
    original = runtime_module.terminate_process_group

    def stop(process, **kwargs):
        attempts.append((process, kwargs))
        if process is failed_process:
            return False
        return original(process, **kwargs)

    monkeypatch.setattr(runtime_module, "terminate_process_group", stop)
    with pytest.raises(RuntimeError, match="did not terminate"):
        manager.close()
    assert [process for process, _ in attempts] == [rtab, nav2]
    assert attempts[0][1] == dict(interrupt_timeout_sec=2.0, terminate_timeout_sec=1.0, kill_timeout_sec=1.0)
    assert attempts[1][1] == dict(interrupt_timeout_sec=5.0, terminate_timeout_sec=2.0, kill_timeout_sec=1.0)
    assert not manager._closed
    owner = manager if failed_runtime == "rtab" else manager.nav2_runtime
    assert owner.process is failed_process
    monkeypatch.setattr(runtime_module, "terminate_process_group", original)
    assert manager.close()
    assert manager.process is manager.nav2_runtime.process is None


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_system_close_waits_for_inflight_launch_then_stops_all_children(runtime, monkeypatch, tmp_path, mode):
    manager, launches, stops = runtime
    (tmp_path / "plant.db").touch()
    entered, release, closing = Event(), Event(), Event()
    original_launch = runtime_module.subprocess.Popen
    original_shutdown = manager._runtime_executor.shutdown

    def launch(args, **kwargs):
        if args[3] == "lidar_rtab_mapping_launch.py":
            entered.set()
            assert release.wait(2)
        return original_launch(args, **kwargs)

    def shutdown(**kwargs):
        closing.set()
        return original_shutdown(**kwargs)

    monkeypatch.setattr(runtime_module.subprocess, "Popen", launch)
    monkeypatch.setattr(manager._runtime_executor, "shutdown", shutdown)
    start = manager.start_mapping if mode == "mapping" else manager.start_localization
    manager.begin_runtime_operation("start", start, "plant")
    with ThreadPoolExecutor(max_workers=1) as closer:
        try:
            assert entered.wait(1)
            future = closer.submit(manager.close)
            assert closing.wait(1)
            assert not future.done()
        finally:
            release.set()
        assert future.result(timeout=2)
    assert manager._closed and manager.nav2_runtime._closed
    assert manager.process is manager.nav2_runtime.process is None
    assert len(launches) == len(stops) == (1 if mode == "mapping" else 2)
    manager._call_service.assert_not_called()  # System close preserves no-save semantics.


def test_start_waiting_to_launch_is_rejected_when_system_is_closing(runtime, monkeypatch):
    manager, launches, stops = runtime
    entered, release, closing = Event(), Event(), Event()
    original_shutdown = manager._runtime_executor.shutdown

    def pending_start():
        entered.set()
        assert release.wait(2)
        return manager.start_mapping("plant")

    def shutdown(**kwargs):
        closing.set()
        return original_shutdown(**kwargs)

    monkeypatch.setattr(manager._runtime_executor, "shutdown", shutdown)
    manager.begin_runtime_operation("start", pending_start)
    with ThreadPoolExecutor(max_workers=1) as closer:
        try:
            assert entered.wait(1)
            future = closer.submit(manager.close)
            assert closing.wait(1)
        finally:
            release.set()
        assert future.result(timeout=2)
    assert launches == []
    assert manager._closed


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_change_map_preserves_mode_and_restarts_only_expected_children(runtime, tmp_path, mode):
    manager, launches, stops = runtime
    (tmp_path / "one.db").touch()
    (tmp_path / "two.db").touch()
    start = manager.start_mapping if mode == "mapping" else manager.start_localization
    first = start("one")
    assert manager.change_map("two")
    assert not first.alive
    assert manager.bb.active_map_name == "two"
    assert manager.get_running_mode() == mode
    assert manager.nav2_runtime.is_running() == (mode == "localization")
    assert len(launches) == (2 if mode == "mapping" else 4)


def test_idle_map_selection_does_not_launch_processes(runtime):
    manager, launches, stops = runtime
    assert manager.change_map("plant")
    assert launches == []
    assert manager.bb.active_map_name == "plant"
    assert manager.get_running_mode() == manager.MODE_NONE


def test_runtime_status_reports_unexpected_exit_and_partial_localization(runtime):
    from diagnostic_msgs.msg import DiagnosticStatus

    manager, _, _ = runtime
    manager.start_mapping("plant")
    manager._publish_runtime_status()
    status = manager._status_pub.publish.call_args.args[0]
    assert status.level == DiagnosticStatus.OK
    assert {item.key: item.value for item in status.values}["mode"] == "mapping"
    manager.set_mode_localization()
    manager._publish_runtime_status()
    status = manager._status_pub.publish.call_args.args[0]
    assert status.level == DiagnosticStatus.ERROR
    assert "Nav2 is not running" in status.message
    assert {item.key: item.value for item in status.values}["mode"] == "localization"
    manager.process.alive = False
    manager._publish_runtime_status()
    status = manager._status_pub.publish.call_args.args[0]
    assert status.level == DiagnosticStatus.ERROR
    assert "exited unexpectedly" in status.message
    assert {item.key: item.value for item in status.values}["mode"] == "none"
    timer = manager._status_timer
    manager.close()
    manager.node.destroy_timer.assert_called_once_with(timer)
