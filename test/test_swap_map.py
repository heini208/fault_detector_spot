"""Tests for map-swap command lifecycle safety."""

from types import SimpleNamespace

import py_trees

from fault_detector_spot.mapping.behaviours.swap_map import SwapMap


class _Helper:
    def __init__(self):
        self.calls = []
        self.failure = None
        self.result = True

    def change_map(self, map_name):
        self.calls.append(map_name)
        if self.failure is not None:
            raise self.failure
        return self.result


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
    return behavior


def test_swap_map_registers_active_map_blackboard_access():
    behavior = SwapMap(_Helper())

    assert "/active_map_name" in behavior.blackboard.read
    assert "/last_command" in behavior.blackboard.read


def test_swap_map_returns_failure_when_helper_raises():
    helper = _Helper()
    helper.failure = RuntimeError("map switch failed")
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "map switch failed" in behavior.feedback_message


def test_swap_map_returns_success_after_switch():
    helper = _Helper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert helper.calls == ["map_b"]
