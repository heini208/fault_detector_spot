"""Offline ACM and service sequencing tests; no ROS nodes or robot commands."""

from concurrent.futures import Future
from copy import deepcopy
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint

from fault_detector_spot.manipulation.moveit_arm_planner import (
    ARM_JOINT_NAMES, MoveItArmPlanner, MoveItPlanOutcome,
)
from fault_detector_spot.manipulation.moveit_collision_policy import (
    OCTOMAP_OBJECT_ID, occupancy_collision_matrix,
)


def allowed(matrix, first, second):
    """Evaluate the documented MoveIt explicit-pair/default precedence."""
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
        default_entry_names=["head", "finger"],
        default_entry_values=[False, True],
    )


@pytest.mark.parametrize("ignore", [False, True])
def test_only_occupancy_effective_rules_change_including_default_only_bodies(ignore):
    original = scene_matrix()
    saved = deepcopy(original)
    updated = occupancy_collision_matrix(original, ignore_environment_collisions=ignore)
    names = ["arm", "body", "wall", "head", "finger", "unnamed_link"]
    for first in names:
        for second in names:
            assert allowed(updated, first, second) == allowed(original, first, second)
        assert allowed(updated, first, OCTOMAP_OBJECT_ID) is ignore
        assert allowed(updated, OCTOMAP_OBJECT_ID, first) is ignore
    assert original == saved
    assert not allowed(updated, "arm", "body")
    assert not allowed(updated, "head", "wall")


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


class Client:
    def __init__(self):
        self.available = True
        self.requests = []
        self.futures = []

    def wait_for_service(self, timeout_sec):
        assert timeout_sec == 0.0
        return self.available

    def call_async(self, request):
        self.requests.append(deepcopy(request))
        future = Future()
        self.futures.append(future)
        return future


class Node:
    def __init__(self):
        self.clients = {}
        self.destroyed = []

    def create_client(self, service, name):
        client = Client()
        self.clients[service] = client
        return client

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None)

    def destroy_client(self, client):
        self.destroyed.append(client)


@pytest.fixture
def rig():
    node = Node()
    clock = [0.0]
    planner = MoveItArmPlanner(
        node, environment_collision_policy_enabled=True,
        monotonic_clock=lambda: clock[0],
    )
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.position.x = 0.42
    target.pose.orientation.w = 1.0
    return planner, node.clients, target, clock


def read_scene(clients, matrix=None):
    response = GetPlanningScene.Response()
    response.scene.allowed_collision_matrix = scene_matrix() if matrix is None else matrix
    clients[GetPlanningScene].futures[-1].set_result(response)


def apply_scene(clients, success=True):
    clients[ApplyPlanningScene].futures[-1].set_result(
        ApplyPlanningScene.Response(success=success),
    )


def plan_response(mode, code=MoveItErrorCodes.SUCCESS, fraction=1.0):
    if mode == "cartesian":
        response = GetCartesianPath.Response()
        response.error_code.val = code
        response.fraction = fraction
        trajectory = response.solution.joint_trajectory
    else:
        response = GetMotionPlan.Response()
        response.motion_plan_response.error_code.val = code
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
    trajectory.joint_names = list(ARM_JOINT_NAMES)
    point = JointTrajectoryPoint(positions=[0.0] * 6)
    point.time_from_start.sec = 1
    trajectory.points = [point]
    return response


@pytest.mark.parametrize("mode", ["motion", "cartesian"])
@pytest.mark.parametrize("ignore", [False, True])
def test_policy_is_applied_before_unchanged_planning_request(rig, mode, ignore):
    planner, clients, target, _ = rig
    start = planner.start if mode == "motion" else planner.start_cartesian
    service = GetMotionPlan if mode == "motion" else GetCartesianPath
    expected = (planner._build_request(target) if mode == "motion"
                else planner._build_cartesian_request(target))
    assert start(target, ignore_environment_collisions=ignore).outcome is MoveItPlanOutcome.RUNNING
    # Freeze the requested target at admission, not when apply-scene finishes.
    target.pose.position.x = 99.0
    assert not clients[service].requests
    read_scene(clients)
    assert planner.poll().outcome is MoveItPlanOutcome.RUNNING
    update = clients[ApplyPlanningScene].requests[-1].scene
    assert update.is_diff and update.robot_state.is_diff
    assert not update.robot_state.attached_collision_objects
    assert not update.world.collision_objects and not update.world.octomap.octomap.data
    assert not update.link_padding and not update.link_scale
    assert allowed(update.allowed_collision_matrix, "arm", OCTOMAP_OBJECT_ID) is ignore
    assert not clients[service].requests
    apply_scene(clients)
    assert planner.poll().outcome is MoveItPlanOutcome.RUNNING
    assert clients[service].requests == [expected]
    response = plan_response(mode)
    clients[service].futures[-1].set_result(response)
    result = planner.poll()
    assert result.outcome is MoveItPlanOutcome.SUCCESS
    assert result.trajectory is not None
    assert not planner.active


def test_checked_request_after_bypass_explicitly_reenables_occupancy(rig):
    planner, clients, target, _ = rig
    matrix = scene_matrix()
    for ignore in (True, False):
        planner.start(target, ignore_environment_collisions=ignore)
        read_scene(clients, matrix)
        planner.poll()
        matrix = clients[ApplyPlanningScene].requests[-1].scene.allowed_collision_matrix
        assert allowed(matrix, "head", OCTOMAP_OBJECT_ID) is ignore
        apply_scene(clients)
        planner.poll()
        clients[GetMotionPlan].futures[-1].set_result(plan_response("motion"))
        assert planner.poll().outcome is MoveItPlanOutcome.SUCCESS


@pytest.mark.parametrize("stage", ["read", "apply", "plan"])
@pytest.mark.parametrize("timeout", [False, True])
def test_cancel_or_timeout_drains_request_without_next_stage_or_late_result(rig, stage, timeout):
    planner, clients, target, clock = rig
    planner.start(target, ignore_environment_collisions=True)
    service = GetPlanningScene
    if stage in ("apply", "plan"):
        read_scene(clients)
        planner.poll()
        service = ApplyPlanningScene
    if stage == "plan":
        apply_scene(clients)
        planner.poll()
        service = GetMotionPlan
    future = clients[service].futures[-1]
    counts = {key: len(value.requests) for key, value in clients.items()}
    if timeout:
        clock[0] = planner.response_timeout_sec + 1.0
        assert planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    else:
        planner.cancel()
    assert not future.cancelled()
    assert planner.active
    assert planner.start_cartesian(target).outcome is MoveItPlanOutcome.ERROR
    if stage == "read":
        read_scene(clients)
    elif stage == "apply":
        apply_scene(clients)
    else:
        future.set_result(plan_response("motion"))
    assert planner.poll().trajectory is None
    assert not planner.active
    assert counts == {key: len(value.requests) for key, value in clients.items()}
    assert planner.start_cartesian(target).outcome is MoveItPlanOutcome.RUNNING


def test_apply_rejection_does_not_submit_plan_and_next_request_can_retry(rig):
    planner, clients, target, _ = rig
    planner.start(target)
    read_scene(clients)
    planner.poll()
    apply_scene(clients, success=False)
    assert planner.poll().outcome is MoveItPlanOutcome.FAILURE
    assert not clients[GetMotionPlan].requests
    assert not planner.active
    assert planner.start(target).outcome is MoveItPlanOutcome.RUNNING


def test_uncertain_apply_completion_blocks_further_planning(rig):
    planner, clients, target, _ = rig
    planner.start(target)
    read_scene(clients)
    planner.poll()
    clients[ApplyPlanningScene].futures[-1].set_exception(RuntimeError("transport lost"))
    assert planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert planner.active
    assert planner.start(target).outcome is MoveItPlanOutcome.SERVICE_UNAVAILABLE
    assert not clients[GetMotionPlan].requests


@pytest.mark.parametrize("mode,code,fraction", [
    ("motion", MoveItErrorCodes.PLANNING_FAILED, 1.0),
    ("cartesian", MoveItErrorCodes.SUCCESS, 0.5),
])
def test_existing_plan_failure_and_partial_path_rejection_remain(rig, mode, code, fraction):
    planner, clients, target, _ = rig
    (planner.start if mode == "motion" else planner.start_cartesian)(target)
    read_scene(clients)
    planner.poll()
    apply_scene(clients)
    planner.poll()
    service = GetMotionPlan if mode == "motion" else GetCartesianPath
    clients[service].futures[-1].set_result(plan_response(mode, code, fraction))
    update = planner.poll()
    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert update.trajectory is None


def test_default_planner_does_not_add_scene_dependencies():
    node = Node()
    planner = MoveItArmPlanner(node)
    target = PoseStamped()
    target.header.frame_id = "body"
    target.pose.orientation.w = 1.0
    assert planner.start(target).outcome is MoveItPlanOutcome.RUNNING
    assert set(node.clients) == {GetMotionPlan, GetCartesianPath}
    assert len(node.clients[GetMotionPlan].requests) == 1
    future = node.clients[GetMotionPlan].futures[-1]
    planner.cancel()
    assert future.cancelled()
    assert not planner.active
    assert planner.start(target).outcome is MoveItPlanOutcome.RUNNING


def test_read_failure_can_retry_without_changing_scene(rig):
    planner, clients, target, _ = rig
    planner.start(target)
    clients[GetPlanningScene].futures[-1].set_exception(RuntimeError("read failed"))
    assert planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert not clients[ApplyPlanningScene].requests
    assert not planner.active
    assert planner.start(target).outcome is MoveItPlanOutcome.RUNNING


def test_bypass_requires_enabled_policy():
    node = Node()
    planner = MoveItArmPlanner(node)
    target = PoseStamped()
    target.header.frame_id = "body"
    update = planner.start(target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.ERROR
    assert all(not client.requests for client in node.clients.values())


@pytest.mark.parametrize("service", [GetPlanningScene, ApplyPlanningScene, GetMotionPlan])
def test_missing_required_service_does_not_submit_work(rig, service):
    planner, clients, target, _ = rig
    clients[service].available = False
    assert planner.start(target).outcome is MoveItPlanOutcome.SERVICE_UNAVAILABLE
    assert all(not client.requests for client in clients.values())
