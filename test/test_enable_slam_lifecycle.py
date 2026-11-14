"""Tests for finite nonblocking mapping-start lifecycle."""

from types import SimpleNamespace

import py_trees
import pytest

from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM


class FakeHelper:
    def __init__(self):
        self.running = False
        self.start_calls = 0
        self.start_maps = []
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

    def start_mapping(self, map_name=None, rviz=True):
        self.start_calls += 1
        self.start_maps.append(map_name)
        if map_name is not None:
            self.blackboard.active_map_name = map_name
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
    helper.blackboard = behavior.blackboard
    return behavior


def test_mapping_start_finishes_when_launch_process_is_running():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert helper.start_calls == 1

    assert behavior.update() == py_trees.common.Status.SUCCESS
    assert helper.start_calls == 1
    assert behavior.feedback_message == "Mapping enabled"


@pytest.mark.parametrize("requested_map,active_map", [
    ("new", "old"), ("new", "new"), ("new", None), ("", "selected"),
])
def test_mapping_start_passes_target_directly_to_runtime(requested_map, active_map):
    helper = FakeHelper()
    behavior = _behavior(
        helper,
        active_map=active_map,
        requested_map=requested_map,
    )

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert helper.start_maps == [requested_map or active_map]
    assert helper.change_calls == []
    assert behavior.update() == py_trees.common.Status.SUCCESS


def test_mapping_start_requires_requested_map_before_reporting_success():
    helper = FakeHelper()
    behavior = _behavior(helper)
    assert behavior.update() == py_trees.common.Status.RUNNING
    helper.blackboard.active_map_name = "other"

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "without selecting 'plant'" in behavior.feedback_message


def test_mapping_start_fails_if_process_dies_after_launch():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert behavior.update() == py_trees.common.Status.RUNNING
    helper.running = False

    assert behavior.update() == py_trees.common.Status.FAILURE
    assert "stopped before becoming active" in behavior.feedback_message


def test_mapping_guard_failure_is_reported_for_the_current_request():
    helper = FakeHelper()
    behavior = _behavior(helper)
    behavior.blackboard.last_command.request_id = "mapping-request"
    detail = "Cannot extend existing map 'plant': Localization estimate is stale"

    def reject_mapping(*args, **kwargs):
        raise RuntimeError(detail)

    helper.start_mapping = reject_mapping
    behavior.initialise()
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.FAILURE
    assert behavior.blackboard.command_failure_request_id == "mapping-request"
    assert behavior.blackboard.command_failure_detail == f"Failed to launch mapping: {detail}"

    behavior.initialise()
    assert behavior.blackboard.command_failure_detail == ""
