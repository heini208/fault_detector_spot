"""Share one policy, planner and idle scene refresher across arm commands."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.behaviour_tree.behaviours import (
    robot_command_resources as resources_module,
)


class Node:
    def __init__(self):
        self.values = {}

    def has_parameter(self, name):
        return name in self.values

    def declare_parameter(self, name, value):
        self.values[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


@pytest.fixture
def factories(monkeypatch):
    planner, scene, control, refresh = Mock(), Mock(), Mock(), Mock()
    listener = Mock(side_effect=AssertionError("Collision controls need no TF listener"))
    monkeypatch.setattr(resources_module, "MoveItArmPlanner", planner)
    monkeypatch.setattr(resources_module, "MoveItCollisionScene", scene)
    monkeypatch.setattr(resources_module, "ArmCollisionControl", control)
    monkeypatch.setattr(resources_module, "MoveItSensorHead", Mock())
    monkeypatch.setattr(resources_module, "MoveItSceneRefresh", refresh)
    monkeypatch.setattr(resources_module, "TFListenerWrapper", listener)
    return planner, scene, control, listener, refresh


@pytest.mark.parametrize("early_control", [False, True])
def test_policy_and_planner_share_one_owner_without_mapping_or_tf(factories, early_control):
    planner, scene, control, listener, refresh = factories
    closed = []
    refresh.return_value.destroy.side_effect = lambda: closed.append("refresh")
    planner.return_value.destroy.side_effect = lambda: closed.append("planner")
    resources = resources_module.RobotCommandResources()
    node = Node()
    if early_control:
        assert resources.get_arm_collision_control(node) is control.return_value
        planner.assert_not_called()
        scene.assert_not_called()
        refresh.assert_not_called()
    first = resources.get_moveit_arm_planner(node)
    assert resources.get_moveit_arm_planner(node) is first
    assert resources.get_arm_collision_control(node) is control.return_value
    control.assert_called_once_with(node)
    scene.assert_called_once_with(
        node, control.return_value,
        sensor_head=resources_module.MoveItSensorHead.return_value,
    )
    resources_module.MoveItSensorHead.assert_called_once_with(node)
    assert planner.call_args.kwargs["collision_scene"] is scene.return_value
    planner.assert_called_once()
    refresh.assert_called_once_with(
        node, planner.return_value, resources_module.MoveItSensorHead.return_value,
        resources._arm_motion_idle,
    )
    assert refresh.call_args.args[3]()
    listener.assert_not_called()
    resources.close()
    resources.close()
    planner.return_value.destroy.assert_called_once()
    control.return_value.destroy.assert_called_once()
    refresh.return_value.destroy.assert_called_once()
    assert closed == ["refresh", "planner"]


def test_early_control_closes_without_creating_planner_and_cannot_cross_nodes(factories):
    planner, scene, control, listener, refresh = factories
    resources = resources_module.RobotCommandResources()
    resources.get_arm_collision_control(Node())
    with pytest.raises(RuntimeError, match="across ROS nodes"):
        resources.get_arm_collision_control(Node())
    resources.close()
    resources.close()
    control.return_value.destroy.assert_called_once()
    planner.assert_not_called()
    scene.assert_not_called()
    listener.assert_not_called()
    refresh.assert_not_called()


def test_idle_refresh_checks_execution_even_when_planner_has_finished(factories):
    planner, _, _, _, refresh = factories
    resources = resources_module.RobotCommandResources()
    resources.get_moveit_arm_planner(Node())
    arm_is_idle = refresh.call_args.args[3]
    planner.return_value.active = False
    first = SimpleNamespace(active=False, shutdown=Mock())
    second = SimpleNamespace(active=True, shutdown=Mock())
    resources._arm_movement_executors = {"first": first, "second": second}

    assert not arm_is_idle()
    second.active = False
    assert arm_is_idle()
    first.active = True
    assert not arm_is_idle()

    resources.close()


def test_planning_budget_override_reaches_shared_planner(factories):
    planner, _, _, _, _ = factories
    resources = resources_module.RobotCommandResources()
    node = Node()
    node.values["arm.motion.moveit_allowed_planning_time_sec"] = 24.0

    resources.get_moveit_arm_planner(node)

    assert planner.call_args.kwargs["allowed_planning_time_sec"] == 24.0
    resources.close()
