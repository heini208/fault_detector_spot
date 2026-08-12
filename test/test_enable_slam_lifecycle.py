"""Tests for finite mapping-start lifecycle."""

from types import SimpleNamespace

import py_trees

from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM


class FakeHelper:
    def __init__(self):
        self.running = False
        self.start_calls = 0
        self.change_calls = []

    def change_map(self, map_name):
        self.change_calls.append(map_name)
        return True

    def start_mapping_from_existing(self):
        self.start_calls += 1
        self.running = True
        return SimpleNamespace(poll=lambda: None)

    def is_rtabmap_running(self):
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


def test_mapping_start_does_not_wait_for_map_topic_data():
    helper = FakeHelper()
    behavior = _behavior(helper)

    assert not hasattr(behavior, "map_received_after_launch")
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.SUCCESS


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
