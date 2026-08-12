"""Tests for finite nonblocking mapping-start lifecycle."""

from types import SimpleNamespace

import py_trees

from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM


class FakeHelper:
    def __init__(self):
        self.running = False
        self.start_calls = 0
        self.change_calls = []
        self._operation = None
        self._result = None
        self._error = None

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
        self.change_calls.append(map_name)
        return True

    def start_mapping_from_existing(self):
        self.start_calls += 1
        self.running = True
        return SimpleNamespace(poll=lambda: None)

    def is_mapping_running(self):
        return self.running


def _behavior(helper, active_map="plant", requested_map="plant"):
    behavior = EnableSLAM(helper)
    behavior.blackboard = SimpleNamespace(
        active_map_name=active_map,
        last_command=SimpleNamespace(map_name=requested_map),
    )
    return behavior


def test_mapping_start_finishes_when_launch_process_is_running():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert helper.start_calls == 1

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert helper.start_calls == 1
    assert behavior.feedback_message == "Mapping enabled"


def test_mapping_start_does_not_block_for_runtime_operation():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._launch_requested


def test_mapping_start_synchronizes_requested_map_before_launch():
    helper = FakeHelper()
    behavior = _behavior(
        helper,
        active_map="old",
        requested_map="new",
    )

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert helper.change_calls == ["new"]


def test_mapping_start_fails_if_process_dies_after_launch():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    helper.running = False

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "stopped before becoming active" in behavior.feedback_message
