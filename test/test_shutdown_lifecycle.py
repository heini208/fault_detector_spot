"""Regression tests for quiet, ownership-based application shutdown."""

import inspect
from pathlib import Path
from threading import RLock
from types import SimpleNamespace

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


ROOT = Path(__file__).parents[1]


class _Executor:
    """Record executor shutdown requests."""

    def __init__(self):
        self.shutdown_calls = 0

    def shutdown(self, **kwargs):
        assert kwargs == {'wait': True, 'cancel_futures': True}
        self.shutdown_calls += 1


class _Nav2Helper:
    """Record Nav2 stop requests."""

    def __init__(self):
        self.stop_calls = 0

    def stop(self):
        self.stop_calls += 1
        return True


def test_idle_runtime_close_is_immediate_and_idempotent():
    helper = RTABHelper.__new__(RTABHelper)
    helper.bb = SimpleNamespace(slam_launch_process=None)
    helper.nav2_helper = _Nav2Helper()
    helper._runtime_lock = RLock()
    helper._runtime_executor = _Executor()
    helper._runtime_future = None
    helper._runtime_operation = ''
    helper._closing = False
    helper._executor_closed = False
    helper._closed = False

    assert helper.close()
    assert helper.close()
    assert helper._runtime_executor.shutdown_calls == 1
    assert helper.nav2_helper.stop_calls == 1


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
