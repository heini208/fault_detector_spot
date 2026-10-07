"""Occupancy policy contracts without ROS nodes or robot commands."""

from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import Mock

from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, CollisionObject,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest

from fault_detector_spot.manipulation.moveit_collision_policy import (
    OCTOMAP_OBJECT_ID, occupancy_collision_matrix,
)
from fault_detector_spot.manipulation.moveit_collision_scene import MoveItCollisionScene


def allowed(matrix, first, second):
    """Evaluate MoveIt's explicit-pair/default collision rule precedence."""
    if first in matrix.entry_names and second in matrix.entry_names:
        return matrix.entry_values[matrix.entry_names.index(first)].enabled[
            matrix.entry_names.index(second)
        ]
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    if first in defaults and second in defaults:
        return defaults[first] and defaults[second]
    return defaults.get(first, defaults.get(second, False))


def scene_matrix():
    return AllowedCollisionMatrix(
        entry_names=["arm", "body", "wall", OCTOMAP_OBJECT_ID],
        entry_values=[AllowedCollisionEntry(enabled=row) for row in [
            [True, False, False, False],
            [False, True, False, False],
            [False, False, True, True],
            [False, False, True, False],
        ]],
        default_entry_names=["probe", "finger"],
        default_entry_values=[False, True],
    )


@pytest.mark.parametrize("ignore", [False, True])
def test_only_occupancy_rules_change_including_default_only_and_new_bodies(ignore):
    original = scene_matrix()
    saved = deepcopy(original)
    updated = occupancy_collision_matrix(original, ignore_environment_collisions=ignore)
    names = ["arm", "body", "wall", "probe", "finger", "new_link"]
    for first in names:
        for second in names:
            assert allowed(updated, first, second) == allowed(original, first, second)
        assert allowed(updated, first, OCTOMAP_OBJECT_ID) is ignore
        assert allowed(updated, OCTOMAP_OBJECT_ID, first) is ignore
    assert original == saved
    assert not allowed(updated, "arm", "body")
    assert not allowed(updated, "probe", "wall")


def test_empty_matrix_and_reenable_after_bypass():
    matrix = AllowedCollisionMatrix()
    for ignore in (True, False, True, False):
        matrix = occupancy_collision_matrix(matrix, ignore_environment_collisions=ignore)
        assert allowed(matrix, "new_robot_link", OCTOMAP_OBJECT_ID) is ignore
        assert not allowed(matrix, "new_robot_link", "another_link")


@pytest.mark.parametrize("malformation", ["row", "duplicate", "asymmetric", "defaults"])
def test_invalid_matrix_is_rejected(malformation):
    matrix = scene_matrix()
    if malformation == "row":
        matrix.entry_values.pop()
    elif malformation == "duplicate":
        matrix.entry_names[0] = "body"
    elif malformation == "asymmetric":
        matrix.entry_values[0].enabled[1] = True
    else:
        matrix.default_entry_values.pop()
    with pytest.raises(ValueError):
        occupancy_collision_matrix(matrix, ignore_environment_collisions=True)


@dataclass(frozen=True)
class PolicyState:
    enabled: bool = False
    revision: int = 0


class Node:
    def __init__(self):
        self.clients = {}

    def create_client(self, service_type, name):
        client = SimpleNamespace(service_type=service_type, name=name)
        self.clients[name] = client
        return client

    def destroy_client(self, client):
        del self.clients[client.name]


def adapter(enabled=False):
    node = Node()
    control = SimpleNamespace(state=Mock(return_value=PolicyState(enabled=enabled)))
    return MoveItCollisionScene(node, control), control, node


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("ignore", [False, True])
def test_scene_policy_applies_global_setting_and_command_bypass_without_geometry(enabled, ignore):
    scene, control, node = adapter(enabled)
    phase, client, request = scene.prepare(ignore)
    assert phase == "read_scene"
    assert client is node.clients["/get_planning_scene"]
    assert request.components.components == PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    assert control.state.call_count == (0 if ignore else 1)

    response = GetPlanningScene.Response()
    response.scene.allowed_collision_matrix = scene_matrix()
    response.scene.world.collision_objects = [CollisionObject(id="wall")]
    original = deepcopy(response)
    phase, client, request = scene.apply_request(response)
    assert phase == "apply"
    assert client is node.clients["/apply_planning_scene"]
    assert isinstance(request, ApplyPlanningScene.Request)
    assert request.scene.is_diff and request.scene.robot_state.is_diff
    assert not request.scene.world.collision_objects
    assert not request.scene.world.octomap.octomap.data
    assert not request.scene.robot_state.attached_collision_objects
    assert allowed(request.scene.allowed_collision_matrix, "arm", OCTOMAP_OBJECT_ID) is (
        ignore or not enabled
    )
    assert not allowed(request.scene.allowed_collision_matrix, "arm", "wall")
    assert not allowed(request.scene.allowed_collision_matrix, "arm", "body")
    assert response == original


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("change", ["enabled", "revision"])
def test_changed_global_policy_invalidates_preparation_even_after_toggle_roundtrip(enabled, change):
    scene, control, _ = adapter(enabled)
    scene.prepare(False)
    control.state.return_value = PolicyState(
        enabled=not enabled if change == "enabled" else enabled,
        revision=0 if change == "enabled" else 2,
    )
    with pytest.raises(RuntimeError, match="setting changed"):
        scene.validate()
    with pytest.raises(RuntimeError, match="setting changed"):
        scene.apply_request(GetPlanningScene.Response())


def test_explicit_bypass_does_not_depend_on_global_policy():
    scene, control, _ = adapter()
    control.state.side_effect = AssertionError("Explicit bypass must not read global policy")
    scene.prepare(True)
    scene.validate()
    _, _, request = scene.apply_request(GetPlanningScene.Response())
    assert allowed(request.scene.allowed_collision_matrix, "arm", OCTOMAP_OBJECT_ID)
    control.state.assert_not_called()


@pytest.mark.parametrize("value", [None, 0, 1, "false"])
def test_nonboolean_bypass_is_rejected(value):
    scene, control, _ = adapter()
    with pytest.raises(TypeError, match="boolean"):
        scene.prepare(value)
    with pytest.raises(TypeError, match="boolean"):
        occupancy_collision_matrix(AllowedCollisionMatrix(), ignore_environment_collisions=value)
    control.state.assert_not_called()


def test_adapter_owns_only_scene_read_and_apply_clients():
    scene, _, node = adapter()
    assert set(node.clients) == {"/get_planning_scene", "/apply_planning_scene"}
    scene.destroy()
    assert not node.clients
