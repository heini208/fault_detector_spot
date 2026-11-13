"""Regression tests for quiet, ownership-based application shutdown."""

import inspect
import time
from concurrent.futures import Future
from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager
from fault_detector_spot.mapping.runtime.mapping_safety_monitor import MappingSafetyMonitor
from fault_detector_spot.shared.ros.shutdown_coordinator import ShutdownCoordinator


ROOT = Path(__file__).parents[1]


class _Executor:
    """Record executor shutdown requests."""

    def __init__(self):
        self.shutdown_calls = 0

    def shutdown(self, **kwargs):
        assert kwargs == {'wait': True, 'cancel_futures': True}
        self.shutdown_calls += 1


class _Nav2RuntimeManager:
    """Record Nav2 stop requests."""

    def __init__(self):
        self.stop_calls = 0

    def stop(self):
        self.stop_calls += 1
        return True

    def close(self):
        return self.stop()


def test_idle_runtime_close_is_immediate_and_idempotent():
    helper = RtabmapRuntimeManager.__new__(RtabmapRuntimeManager)
    helper.bb = SimpleNamespace(slam_launch_process=None)
    helper.nav2_runtime = _Nav2RuntimeManager()
    helper._runtime_lock = RLock()
    helper._process_lock = RLock()
    helper._runtime_executor = _Executor()
    helper._runtime_future = None
    helper._runtime_operation = ''
    helper._closing = False
    helper._executor_closed = False
    helper._closed = False
    helper._status_timer = None
    helper._safety = MappingSafetyMonitor(Mock(), helper)

    assert helper.close()
    assert helper.close()
    assert helper._runtime_executor.shutdown_calls == 1
    assert helper.nav2_runtime.stop_calls == 1


def test_runner_preserves_ros_context_for_ordered_shutdown():
    source = inspect.getsource(runner)

    assert 'signal.signal' not in source
    assert 'signal_handler_options=SignalHandlerOptions.NO' in source
    assert 'with shutdown_signals()' in source
    assert 'cleanup["ROS context"] = rclpy.try_shutdown' in source


@pytest.mark.parametrize("executable", ["bt_runner", "application_api"])
def test_launch_has_time_to_finish_bounded_child_cleanup(executable):
    source = (ROOT / 'launch' / 'fault_detector_launch.py').read_text(
        encoding='utf-8'
    )
    bt_runner = source[source.index(f'executable="{executable}"'):]
    bt_runner = bt_runner[:bt_runner.index('),')]

    assert 'sigterm_timeout="45.0"' in bt_runner
    assert 'sigkill_timeout="5.0"' in bt_runner


def test_helper_cleanup_continues_after_error_and_retains_failed_runtime_for_retry():
    from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import HelperInitializer

    helper = HelperInitializer.__new__(HelperInitializer)
    helper.rtabmap_runtime = Mock()
    helper.rtabmap_runtime.close.side_effect = RuntimeError("process did not terminate")
    lidar = Mock()
    helper.lidar_adapter_runtime = lidar
    tags = Mock()
    helper.tag_state_source = tags
    shutdown = ShutdownCoordinator(
        close_admission={}, request_stop={}, confirmations={},
        cleanup=helper.cleanup_operations(), progress=Event().wait,
        report_unconfirmed=Mock(), timeout_sec=1.0, cleanup_timeout_sec=1.0,
    )
    report = shutdown.close()
    assert report.cleanup_errors == {"rtabmap_runtime": "process did not terminate"}
    helper.rtabmap_runtime.close.assert_called_once_with()
    lidar.close.assert_called_once_with()
    tags.destroy.assert_called_once_with()
    assert helper.tag_state_source is None
    helper.rtabmap_runtime.close.side_effect = None
    assert not shutdown.close().cleanup_errors
    assert helper.rtabmap_runtime is None
    lidar.close.assert_called_once_with()
    tags.destroy.assert_called_once_with()


@pytest.mark.parametrize("missing", [None, "acceptance", "result", "physical_stop"])
def test_active_runner_shutdown_sends_one_stop_without_action_cancellation_or_waiting_for_robot(monkeypatch, missing):
    from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources
    from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
    from fault_detector_spot.navigation.base_pose_source import BasePoseSample
    from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
    from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
    from test_execution_stop_evidence import base_executor
    from test_base_movement_executor import FakeGoalHandle
    from test_arm_movement_executor import executor_with_client, FakeTransformer

    owner, accepted, clock, source = base_executor()
    owner.sit()
    result = Future()
    handle = FakeGoalHandle(result)
    if missing != "acceptance":
        accepted.set_result(handle)
        owner.poll()
    if missing not in {"acceptance", "result"}:
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    if missing != "physical_stop":
        source.value = BasePoseSample(
            2.0, 3.0, 0.4, clock.now,
            PoseData(Vector3Data(2.0, 3.0, 0.5), quaternion_from_euler("xyz", [0, 0, 0.4])),
        )
    assert owner.active
    resources = RobotCommandResources(monotonic_clock=clock)
    resources._base_movement_executors[""] = owner
    events = []
    stop_reply = Future()
    stop_service = Mock()
    stop_service.wait_for_service.return_value = True
    stop_service.call_async.side_effect = lambda _request: (events.append("shutdown Stop"), stop_reply)[1]
    arm, _ = executor_with_client(FakeTransformer(), arm_stop_service_client=stop_service)
    resources._arm_movement_executors[""] = arm
    velocity_publisher = Mock()
    resources._velocity_publisher = velocity_publisher
    resources._node = SimpleNamespace(
        destroy_publisher=lambda publisher: events.append("velocity publisher destroyed"),
    )
    client = Mock()
    client.destroy.side_effect = lambda: events.append("client destroyed")
    resources._clients["robot"] = client
    tree = SimpleNamespace(root=SimpleNamespace(iterate=lambda: ()),
                           timer=Mock(), shutdown=lambda **kwargs: events.append("tree destroyed"))

    def runtime_close():
        assert not owner.execution_idle
        assert "bt_runner/base/default" in StopUnconfirmedStore().load()
        assert "executor stopped" not in events
        events.append("runtime stopped")

    helper = SimpleNamespace(robot_command_resources=resources,
                             cleanup_operations=lambda: {"runtime": runtime_close})
    monkeypatch.setattr(runner, "helper_initializer", helper)
    monkeypatch.setattr(runner.rclpy, "try_shutdown", lambda: events.append("ROS stopped"))
    node = SimpleNamespace(
        has_parameter=lambda name: True,
        get_parameter=lambda name: SimpleNamespace(value=2.0),
        get_logger=lambda: Mock(),
        destroy_node=lambda: events.append("node destroyed"),
    )

    def callbacks(timeout_sec):
        Event().wait(min(timeout_sec, 0.001))
        clock.now += timeout_sec

    executor = SimpleNamespace(
        spin_once=callbacks, shutdown=lambda **kwargs: events.append("executor stopped"),
    )
    shutdown = runner.make_shutdown_coordinator(node, tree, executor)
    shutdown._timeout = 5.0
    started = time.monotonic()
    report = shutdown.close()
    assert not report.cleanup_errors
    assert not report.confirmed
    assert "base/default" in report.unresolved
    assert "command sources" in report.unresolved
    assert not owner.execution_idle
    assert "bt_runner/base/default" in StopUnconfirmedStore().load()
    assert time.monotonic() - started < 1.0
    assert handle.cancel_calls == 0
    assert owner.action_client.send_calls == 1
    velocity_publisher.publish.assert_not_called()
    stop_service.call_async.assert_called_once()
    stop_service.wait_for_service.assert_called_once_with(timeout_sec=0.0)
    assert not stop_reply.done()
    tree.timer.cancel.assert_called_once()
    assert events == ["shutdown Stop", "runtime stopped", "velocity publisher destroyed", "client destroyed",
                      "tree destroyed", "executor stopped", "node destroyed", "ROS stopped"]
    # Late callbacks cannot turn detached ownership into cancellation or erase
    # the shutdown record, even when all action evidence eventually arrives.
    if not accepted.done():
        accepted.set_result(handle)
    if not result.done():
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    owner.poll()
    assert handle.cancel_calls == 0
    assert owner.action_client.send_calls == 1
    velocity_publisher.publish.assert_not_called()
    assert "bt_runner/base/default" in StopUnconfirmedStore().load()
    snapshot = list(events)
    assert shutdown.close() == report
    assert events == snapshot
    stop_reply.set_result(SimpleNamespace(success=True, message="accepted"))
    assert shutdown.close() == report
    stop_service.call_async.assert_called_once()


def test_resource_cleanup_retains_failed_client_and_releases_state_source():
    from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources
    resources = RobotCommandResources()
    client = Mock()
    client.destroy.side_effect = [RuntimeError("client busy"), None]
    state_source = Mock()
    resources._clients["robot"] = client
    resources._arm_state_source = state_source
    with pytest.raises(RuntimeError, match="client busy"):
        resources.close()
    assert resources._clients["robot"] is client
    assert resources._arm_state_source is None
    resources.close()
    resources.close()
    assert client.destroy.call_count == 2
    state_source.destroy.assert_called_once()
