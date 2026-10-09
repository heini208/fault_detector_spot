"""Reject waypoint/map mismatches before starting navigation preparation."""

from types import SimpleNamespace
from unittest.mock import Mock

import py_trees
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.mapping.model.models import MapDefinition, Waypoint
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.navigation.commands.waypoint_command import WaypointCommand
from fault_detector_spot.navigation.waypoint_navigation_executor import (
    WaypointNavigationOutcome,
    WaypointNavigationUpdate,
)
from fault_detector_spot.shared.geometry.models import PoseData


@pytest.fixture
def waypoint_tree(tmp_path, monkeypatch):
    py_trees.blackboard.Blackboard.clear()
    repository = MapRepository(tmp_path)
    for map_id, x in (("plant_a", 1.0), ("plant_b", 8.0)):
        pose = PoseData.identity()
        pose.position.x = x
        repository.save(map_id, MapDefinition(
            map_id=map_id, display_name=map_id,
            waypoints=[Waypoint("inspection", "Inspection", pose)],
        ))
    executor = Mock()
    executor.navigate.return_value = WaypointNavigationUpdate(
        WaypointNavigationOutcome.SUCCESS, "Navigation complete",
    )
    resources = Mock()
    resources.get_waypoint_navigation_executor.return_value = executor
    monkeypatch.setattr(runner, "get_helper_container", lambda _node: SimpleNamespace(
        robot_command_resources=resources,
    ))
    node = Mock()
    node.get_parameter.return_value.value = str(tmp_path)
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=12)
    tree = runner.build_navigate_to_goal_pose_tree(node)
    for child in tree.children:
        child.setup(node=node)
    writer = py_trees.blackboard.Client(name="WaypointMapTest")
    for key in ("last_command", "active_map_name"):
        writer.register_key(key, access=py_trees.common.Access.WRITE)
    command = WaypointCommand(CommandID.MOVE_TO_WAYPOINT, Time(), "plant_a", "inspection")
    command.request_id = "waypoint-request"
    writer.last_command = command
    writer.active_map_name = "plant_a"
    yield SimpleNamespace(
        tree=tree, resolver=tree.children[0], executor=executor, writer=writer,
        command=command, repository=repository, resources=resources,
    )
    py_trees.blackboard.Blackboard.clear()


def assert_rejected(rig, detail):
    rig.tree.tick_once()
    assert rig.tree.status is py_trees.common.Status.FAILURE
    assert detail in rig.resolver.feedback_message
    assert rig.command.goal_pose is None
    rig.resources.get_waypoint_navigation_executor.assert_not_called()
    rig.executor.navigate.assert_not_called()
    assert py_trees.blackboard.Blackboard.get("command_failure_request_id") == "waypoint-request"
    assert py_trees.blackboard.Blackboard.get("command_failure_detail") == rig.resolver.feedback_message


def test_same_waypoint_name_in_another_active_map_is_rejected(waypoint_tree):
    rig = waypoint_tree
    rig.command.goal_pose = PoseStamped()  # Discard any previously resolved goal.
    rig.writer.active_map_name = "plant_b"
    assert_rejected(rig, "'plant_a', but the active map is 'plant_b'")


@pytest.mark.parametrize("active_map", [None, "", "missing_key"])
def test_no_active_map_fails_before_navigation(waypoint_tree, active_map):
    rig = waypoint_tree
    if active_map == "missing_key":
        rig.writer.unset("active_map_name")
    else:
        rig.writer.active_map_name = active_map
    assert_rejected(rig, "no active map")


def test_waypoint_must_exist_in_matching_active_map(waypoint_tree):
    rig = waypoint_tree
    rig.repository.delete_waypoint("plant_a", "inspection")
    assert_rejected(rig, "Waypoint 'inspection' not found in map 'plant_a'")


@pytest.mark.parametrize("active_at_creation,active_at_execution", [
    ("plant_a", "plant_a"),
    ("plant_b", "plant_a"),
    ("plant_a", "plant_b"),
])
def test_map_is_checked_when_queued_command_executes(
    waypoint_tree, active_at_creation, active_at_execution,
):
    rig = waypoint_tree
    rig.writer.active_map_name = active_at_creation
    rig.writer.last_command = WaypointCommand(
        CommandID.MOVE_TO_WAYPOINT, Time(), "plant_a", "inspection",
    )
    rig.command = rig.writer.last_command
    rig.command.request_id = "waypoint-request"
    rig.writer.active_map_name = active_at_execution
    if active_at_execution != "plant_a":
        assert_rejected(rig, "active map is 'plant_b'")
        return

    rig.tree.tick_once()
    assert rig.tree.status is py_trees.common.Status.SUCCESS
    rig.executor.navigate.assert_called_once_with(rig.command.goal_pose)
    goal = rig.command.goal_pose
    assert goal.header.frame_id == "map"
    assert goal.header.stamp.sec == 12
    assert goal.pose.position.x == 1.0
