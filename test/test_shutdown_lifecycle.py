"""Regression tests for quiet, ownership-based application shutdown."""

import inspect
from pathlib import Path
from threading import RLock
from types import SimpleNamespace

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


def test_runner_uses_ros_signal_handling_and_idempotent_shutdown():
    source = inspect.getsource(runner)

    assert 'signal.signal' not in source
    assert 'rclpy.try_shutdown()' in source
    assert 'close_helper_container()' in source


def test_bt_runner_has_time_to_finish_bounded_child_cleanup():
    source = (ROOT / 'launch' / 'fault_detector_launch.py').read_text(
        encoding='utf-8'
    )
    bt_runner = source[source.index('executable="bt_runner"'):]
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
    tags = Mock()
    helper.tag_state_source = tags
    helper.robot_command_resources = Mock()
    with pytest.raises(RuntimeError, match="did not terminate"):
        helper.close()
    helper.rtabmap_runtime.close.assert_called_once_with()
    tags.destroy.assert_called_once_with()
    helper.robot_command_resources.close.assert_called_once_with()
    assert helper.tag_state_source is None


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
