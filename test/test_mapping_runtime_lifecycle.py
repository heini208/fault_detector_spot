"""Regression tests for mapping runtime lifecycle safety."""

from types import SimpleNamespace
import inspect

import py_trees

from fault_detector_spot.mapping.behaviours.enable_localization import (
    EnableLocalization,
)
from fault_detector_spot.mapping.behaviours.stop_mapping import StopMapping
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager
from fault_detector_spot.shared.ros import runtime_manager as runtime_module
from threading import RLock
from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager


class _AsyncHelper:
    def __init__(self):
        self.rtab_running = True
        self.mapping = False
        self.localization = True
        self.nav2_runtime = SimpleNamespace(is_running=lambda: True)
        self._operation = None
        self._result = None
        self._error = None
        self.stop_calls = 0
        self.start_localization_calls = 0
        self.change_calls = []

    def begin_runtime_operation(self, name, callback, *args):
        if self._operation is not None:
            return False
        self._operation = name
        try:
            self._result = callback(*args)
        except Exception as exception:
            self._error = exception
        return True

    def poll_runtime_operation(self, name):
        assert name == self._operation
        self._operation = None
        if self._error is not None:
            error = self._error
            self._error = None
            raise error
        result = self._result
        self._result = None
        return result

    def stop(self, save=True):
        self.stop_calls += 1
        self.rtab_running = False
        self.nav2_runtime = SimpleNamespace(is_running=lambda: False)
        return True

    def is_running(self):
        return self.rtab_running

    def change_map(self, map_name):
        self.change_calls.append(map_name)
        return True

    def start_localization(self):
        self.start_localization_calls += 1
        return SimpleNamespace(poll=lambda: None)

    def is_localization_running(self):
        return (
            self.rtab_running
            and self.localization
            and self.nav2_runtime.is_running()
        )


def test_stop_mapping_stops_localization_runtime_too():
    helper = _AsyncHelper()
    behavior = StopMapping(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert helper.stop_calls == 1
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_localization_does_not_succeed_when_rtabmap_is_dead():
    helper = _AsyncHelper()
    behavior = EnableLocalization(helper)
    behavior.blackboard = SimpleNamespace(
        active_map_name="plant",
        last_command=SimpleNamespace(map_name="plant"),
    )

    assert behavior.update() == py_trees.common.Status.RUNNING
    helper.rtab_running = False

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "RTAB-Map stopped" in behavior.feedback_message


def test_running_mode_no_longer_shells_out_to_ros2_param_get():
    source = inspect.getsource(RtabmapRuntimeManager.get_running_mode)

    assert "subprocess.run" not in source
    assert "slam_runtime_mode" in source


def test_nav2_stop_keeps_process_reference_when_termination_fails(
    monkeypatch,
):
    process = SimpleNamespace()
    helper = Nav2RuntimeManager.__new__(Nav2RuntimeManager)
    helper.bb = SimpleNamespace(nav2_launch_process=process)
    helper._process_lock = RLock()
    helper.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            error=lambda _message: None,
            info=lambda _message: None,
        )
    )
    monkeypatch.setattr(
        runtime_module,
        "terminate_process_group",
        lambda *_args, **_kwargs: False,
    )

    assert not helper.stop()
    assert helper.bb.nav2_launch_process is process
