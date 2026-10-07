"""Offline checks for always-wired scene ownership and shared runtime/TF."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.behaviour_tree.behaviours import (
    robot_command_resources as resources_module,
)


class Node:
    def __init__(self, **overrides):
        self.values = overrides

    def has_parameter(self, name):
        return name in self.values

    def declare_parameter(self, name, value):
        self.values[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


@pytest.fixture
def factories(monkeypatch):
    planner, scene, listener = Mock(), Mock(), Mock()
    monkeypatch.setattr(resources_module, "MoveItArmPlanner", planner)
    monkeypatch.setattr(resources_module, "RtabmapCollisionScene", scene)
    monkeypatch.setattr(resources_module, "TFListenerWrapper", listener)
    return planner, scene, listener


def test_scene_uses_existing_runtime_and_shared_tf_once(factories):
    planner, scene, listener = factories
    resources = resources_module.RobotCommandResources()
    node = Node()
    runtime = object()
    resources.bind_rtabmap_runtime(runtime)
    tf_listener = resources.get_tf_listener(node)
    first = resources.get_moveit_arm_planner(node)
    assert resources.get_moveit_arm_planner(node) is first
    scene.assert_called_once_with(node, tf_listener.buffer, runtime)
    listener.assert_called_once_with(node)
    assert planner.call_args.kwargs["collision_scene"] is scene.return_value
    with pytest.raises(RuntimeError, match="another mapping runtime"):
        resources.bind_rtabmap_runtime(object())


def test_planning_requires_authoritative_runtime(factories):
    planner, scene, listener = factories
    resources = resources_module.RobotCommandResources()
    with pytest.raises(RuntimeError, match="requires the mapping runtime"):
        resources.get_moveit_arm_planner(Node())
    planner.assert_not_called()
    scene.assert_not_called()
    listener.assert_not_called()
