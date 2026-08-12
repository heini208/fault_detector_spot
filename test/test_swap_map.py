"""Tests for nonblocking map-swap lifecycle safety."""

from types import SimpleNamespace

import py_trees

from fault_detector_spot.mapping.behaviours.swap_map import SwapMap
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class _Helper:
    def __init__(self):
        self.calls = []
        self.failure = None
        self.result = True
        self.mode = RTABHelper.MODE_NONE
        self._operation = None
        self._result = None
        self._error = None
        self.writer = None

    def get_running_mode(self):
        return self.mode

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

    def change_map(self, map_name):
        self.calls.append(map_name)
        if self.failure is not None:
            raise self.failure
        if self.writer is not None and self.result is not False:
            self.writer.active_map_name = map_name
        return self.result

    def is_mapping_running(self):
        return self.mode == RTABHelper.MODE_MAPPING

    def is_localization_running(self):
        return self.mode == RTABHelper.MODE_LOCALIZATION


def _behavior(helper, requested="map_b", active="map_a"):
    behavior = SwapMap(helper)
    writer = py_trees.blackboard.Client(name="SwapMapTestWriter")
    writer.register_key(
        "last_command",
        access=py_trees.common.Access.WRITE,
    )
    writer.register_key(
        "active_map_name",
        access=py_trees.common.Access.WRITE,
    )
    writer.last_command = SimpleNamespace(map_name=requested)
    writer.active_map_name = active
    helper.writer = writer
    return behavior


def test_swap_map_registers_active_map_blackboard_access():
    behavior = SwapMap(_Helper())

    assert "/active_map_name" in behavior.blackboard.read
    assert "/last_command" in behavior.blackboard.read


def test_swap_map_returns_failure_when_helper_raises():
    helper = _Helper()
    helper.failure = RuntimeError("map switch failed")
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "map switch failed" in behavior.feedback_message


def test_swap_map_returns_success_after_switch():
    helper = _Helper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert helper.calls == ["map_b"]
