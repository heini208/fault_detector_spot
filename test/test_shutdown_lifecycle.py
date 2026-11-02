"""Regression tests for quiet, ownership-based application shutdown."""

import inspect
import time
from concurrent.futures import Future
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager


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
    assert 'close_helper_container()' in source


@pytest.mark.parametrize("executable", ["bt_runner", "application_api"])
def test_launch_has_time_to_finish_bounded_child_cleanup(executable):
    source = (ROOT / 'launch' / 'fault_detector_launch.py').read_text(
        encoding='utf-8'
    )
    bt_runner = source[source.index(f'executable="{executable}"'):]
    bt_runner = bt_runner[:bt_runner.index('),')]

    assert 'sigterm_timeout="45.0"' in bt_runner
    assert 'sigkill_timeout="5.0"' in bt_runner


def test_helper_close_stops_runtimes_and_cleans_ros_resources_even_on_error():
    from unittest.mock import Mock
    import pytest
    from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import HelperInitializer

    helper = HelperInitializer.__new__(HelperInitializer)
    helper.rtabmap_runtime = Mock()
    helper.rtabmap_runtime.close.side_effect = RuntimeError("process did not terminate")
    lidar = Mock()
    helper.lidar_adapter_runtime = lidar
    tags = Mock()
    helper.tag_state_source = tags
    helper.robot_command_resources = Mock()
    with pytest.raises(RuntimeError, match="did not terminate"):
        helper.close()
    helper.rtabmap_runtime.close.assert_called_once_with()
    lidar.close.assert_called_once_with()
    tags.destroy.assert_called_once_with()
    helper.robot_command_resources.close.assert_called_once_with()
    assert helper.tag_state_source is None
    helper.rtabmap_runtime.close.side_effect = None
    helper.close()
    assert helper.rtabmap_runtime is None
    lidar.close.assert_called_once_with()
    tags.destroy.assert_called_once_with()


def test_runner_retains_failed_helper_close_for_retry(monkeypatch):
    from unittest.mock import Mock
    import pytest

    helper = Mock()
    helper.close.side_effect = [RuntimeError("process did not terminate"), True]
    monkeypatch.setattr(runner, "helper_initializer", helper)
    with pytest.raises(RuntimeError, match="did not terminate"):
        runner.close_helper_container()
    assert runner.helper_initializer is helper
    runner.close_helper_container()
    assert runner.helper_initializer is None
    assert helper.close.call_count == 2


@pytest.mark.parametrize("missing", [None, "acceptance", "result", "physical_stop"])
def test_active_goal_callbacks_remain_available_through_runner_confirmation(monkeypatch, missing):
    from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources
    from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
    from fault_detector_spot.navigation.base_pose_source import BasePoseSample
    from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
    from fault_detector_spot.shared.geometry.rotation import quaternion_from_euler
    from test_execution_stop_evidence import base_executor
    from test_base_movement_executor import FakeGoalHandle

    owner, accepted, clock, source = base_executor()
    owner.sit()
    result = Future()
    handle = FakeGoalHandle(result)
    resources = RobotCommandResources(monotonic_clock=clock)
    resources._base_movement_executors[""] = owner
    events = []
    client = Mock()
    client.destroy.side_effect = lambda: events.append("client destroyed")
    resources._clients["robot"] = client
    tree = SimpleNamespace(root=SimpleNamespace(iterate=lambda: ()),
                           timer=Mock(), shutdown=lambda **kwargs: events.append("tree destroyed"))

    def runtime_close():
        assert missing is not None or owner.execution_idle
        if missing is not None:
            assert StopUnconfirmedStore().load()
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
        time.sleep(0.002)
        clock.now += timeout_sec
        if "client destroyed" in events:
            return
        # Simulate delivery from the callback executor, including acceptance
        # arriving after shutdown has already requested cancellation.
        if clock.now >= 0.1 and not accepted.done() and missing != "acceptance":
            assert resources.stop_latched
            tree.timer.cancel.assert_called_once()
            accepted.set_result(handle)
            events.append("acceptance delivered")
        if clock.now >= 0.2 and accepted.done() and not result.done() and missing != "result":
            result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
            events.append("result delivered")
        if clock.now >= 0.3 and missing != "physical_stop":
            source.value = BasePoseSample(
                2.0, 3.0, 0.4, clock.now,
                PoseData(Vector3Data(2.0, 3.0, 0.5), quaternion_from_euler("xyz", [0, 0, 0.4])),
            )

    executor = SimpleNamespace(
        spin_once=callbacks, shutdown=lambda **kwargs: events.append("executor stopped"),
    )
    shutdown = runner.make_shutdown_coordinator(node, tree, executor)
    shutdown._timeout = 0.3
    started = time.monotonic()
    report = shutdown.close()
    assert not report.cleanup_errors
    assert report.confirmed is (missing is None)
    if missing:
        assert "base/default" in report.unresolved
        assert not owner.execution_idle
        assert StopUnconfirmedStore().load()
    else:
        assert handle.cancel_calls >= 1
        assert events.index("result delivered") < events.index("runtime stopped")
        assert not StopUnconfirmedStore().load()
    assert time.monotonic() - started < 2.5
    assert events.index("client destroyed") < events.index("executor stopped") < events.index("ROS stopped")
    snapshot = list(events)
    assert shutdown.close() == report
    assert events == snapshot


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
