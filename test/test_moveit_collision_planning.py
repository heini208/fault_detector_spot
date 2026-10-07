"""Exercise real occupancy-policy/planner sequencing without ROS execution."""

from concurrent.futures import Future
from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint
import pytest

from fault_detector_spot.manipulation.arm_collision_control import ArmCollisionPolicy
from fault_detector_spot.manipulation.moveit_arm_planner import (
    ARM_JOINT_NAMES, MoveItArmPlanner, MoveItPlanOutcome,
)
from fault_detector_spot.manipulation.moveit_collision_policy import OCTOMAP_OBJECT_ID
from fault_detector_spot.manipulation.moveit_collision_scene import MoveItCollisionScene


READ = "/get_planning_scene"
APPLY = "/apply_planning_scene"
PLAN = "/plan_kinematic_path"
CARTESIAN = "/compute_cartesian_path"


class Client:
    def __init__(self, name, events):
        self.name = name
        self.events = events
        self.requests = []
        self.future = None
        self.available = True

    def wait_for_service(self, timeout_sec):
        assert timeout_sec == 0.0
        return self.available

    def call_async(self, request):
        self.requests.append(deepcopy(request))
        self.events.append(self.name)
        self.future = Future()
        return self.future


class Node:
    def __init__(self):
        self.clients = {}
        self.events = []

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None)

    def create_client(self, _service_type, name):
        client = Client(name, self.events)
        self.clients[name] = client
        return client

    def destroy_client(self, client):
        del self.clients[client.name]


class Rig:
    def __init__(self):
        self.node = Node()
        self.clock = 0.0
        self.policy = ArmCollisionPolicy(enabled=False, revision=0)
        control = SimpleNamespace(state=lambda: self.policy)
        self.scene = MoveItCollisionScene(self.node, control)
        self.planner = MoveItArmPlanner(
            self.node, collision_scene=self.scene, monotonic_clock=lambda: self.clock,
        )
        self.target = PoseStamped()
        self.target.header.frame_id = "body"
        self.target.pose.position.x = 0.65
        self.target.pose.orientation.w = 1.0
        self.server_matrix = AllowedCollisionMatrix(
            entry_names=["arm", "body", "wall"],
            entry_values=[AllowedCollisionEntry(enabled=row) for row in [
                [True, False, False], [False, True, False], [False, False, True],
            ]],
        )

    def toggle(self):
        self.policy = ArmCollisionPolicy(not self.policy.enabled, self.policy.revision + 1)

    def reply(self, service, response):
        self.node.clients[service].future.set_result(response)
        return self.planner.poll()

    def read(self):
        response = GetPlanningScene.Response()
        response.scene.allowed_collision_matrix = deepcopy(self.server_matrix)
        return self.reply(READ, response)

    def applied(self, success=True):
        if success:
            self.server_matrix = deepcopy(
                self.node.clients[APPLY].requests[-1].scene.allowed_collision_matrix,
            )
        return self.reply(APPLY, ApplyPlanningScene.Response(success=success))

    def planned(self):
        cartesian = self.planner._planning_mode == "cartesian"
        response = GetCartesianPath.Response() if cartesian else GetMotionPlan.Response()
        if cartesian:
            response.error_code.val = 1
            response.fraction = 1.0
            trajectory = response.solution.joint_trajectory
        else:
            response.motion_plan_response.error_code.val = 1
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint(positions=[0.0] * 6)
        point.time_from_start.sec = 1
        trajectory.points = [point]
        return self.reply(CARTESIAN if cartesian else PLAN, response)

    def advance_to(self, stage, *, bypass=False):
        self.planner.start(self.target, ignore_environment_collisions=bypass)
        if stage in ("apply", "plan", "dispatch"):
            self.read()
        if stage in ("plan", "dispatch"):
            self.applied()
        if stage == "dispatch":
            assert self.planned().outcome is MoveItPlanOutcome.SUCCESS


@pytest.fixture
def rig():
    result = Rig()
    yield result
    result.planner.destroy()


@pytest.mark.parametrize("cartesian", [False, True])
def test_default_off_on_bypass_on_applies_only_occupancy_policy_before_original_plan(rig, cartesian):
    start = rig.planner.start_cartesian if cartesian else rig.planner.start
    service = CARTESIAN if cartesian else PLAN
    expected = (rig.planner._build_cartesian_request(rig.target) if cartesian
                else rig.planner._build_request(rig.target))
    for index, bypass in enumerate((False, False, True, False)):
        if index == 1:
            rig.toggle()
        assert start(rig.target, ignore_environment_collisions=bypass).outcome is MoveItPlanOutcome.RUNNING
        assert rig.node.events[-1] == READ
        assert len(rig.node.clients[service].requests) == index
        rig.read()
        assert rig.node.events[-1] == APPLY
        assert len(rig.node.clients[service].requests) == index
        diff = rig.node.clients[APPLY].requests[-1].scene
        assert diff.is_diff and diff.robot_state.is_diff
        matrix = diff.allowed_collision_matrix
        arm_row = matrix.entry_values[matrix.entry_names.index("arm")].enabled
        assert arm_row[matrix.entry_names.index(OCTOMAP_OBJECT_ID)] is (bypass or index == 0)
        assert not arm_row[matrix.entry_names.index("body")]
        assert not arm_row[matrix.entry_names.index("wall")]
        rig.applied()
        assert rig.node.clients[service].requests[-1] == expected
        if cartesian:
            assert rig.node.clients[service].requests[-1].avoid_collisions
        assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [READ, APPLY, service] * 4


@pytest.mark.parametrize("initially_enabled", [False, True])
@pytest.mark.parametrize("stage", ["read", "apply", "plan", "dispatch"])
def test_global_toggle_invalidates_ordinary_work_before_dispatch(rig, initially_enabled, stage):
    if initially_enabled:
        rig.toggle()
    rig.advance_to(stage)
    rig.toggle()
    events = list(rig.node.events)
    if stage == "dispatch":
        assert "setting changed" in rig.planner.validate_prepared_scene()
    else:
        update = {"read": rig.read, "apply": rig.applied, "plan": rig.planned}[stage]()
        assert update.outcome is MoveItPlanOutcome.FAILURE
        assert update.trajectory is None
    assert rig.node.events == events


def test_explicit_bypass_remains_valid_across_global_policy_revisions(rig):
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    rig.toggle()
    rig.read()
    rig.toggle()
    rig.applied()
    rig.toggle()
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    rig.toggle()
    assert rig.planner.validate_prepared_scene() is None


@pytest.mark.parametrize("cancel", [False, True])
def test_abandoned_scene_read_never_applies_late_response_or_blocks_another_command(rig, cancel):
    rig.advance_to("read")
    previous = rig.planner._future
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 8.0
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    previous.set_result(GetPlanningScene.Response())
    assert rig.planner.poll().outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == [READ, READ]
    rig.read()
    rig.applied()
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [READ, READ, APPLY, PLAN]


@pytest.mark.parametrize("stage", ["apply", "plan"])
@pytest.mark.parametrize("cancel", [False, True])
def test_abandoned_mutation_or_plan_waits_for_actual_server_reply(rig, stage, cancel):
    rig.advance_to(stage)
    previous = rig.planner._future
    events = list(rig.node.events)
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 8.0
        update = rig.planner.poll()
        assert update.outcome is MoveItPlanOutcome.TIMEOUT
        assert "7.0 s" in update.detail
    assert not previous.cancelled()
    assert not rig.planner.active
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.ERROR
    assert "still running" in update.detail
    assert rig.node.events == events
    previous.set_result(ApplyPlanningScene.Response(success=True))
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == events + [READ]


@pytest.mark.parametrize("stage", ["apply", "plan"])
@pytest.mark.parametrize("response_error", [False, True])
@pytest.mark.parametrize("cancel", [False, True])
def test_unknown_mutation_or_plan_completion_locks_out_further_scene_changes(rig, stage, response_error, cancel):
    rig.advance_to(stage)
    if response_error:
        rig.planner._future.set_exception(RuntimeError("transport lost"))
    else:
        rig.planner._future.set_result(None)
    events = list(rig.node.events)
    if cancel:
        rig.planner.cancel()
    else:
        assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.ERROR
    assert "completion is unknown" in update.detail
    assert rig.node.events == events


def test_read_failure_does_not_create_uncertain_mutation_lockout(rig):
    rig.advance_to("read")
    rig.planner._future.set_exception(RuntimeError("read failed"))
    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == [READ, READ]


def test_rejected_policy_never_submits_a_plan(rig):
    rig.advance_to("apply")
    assert rig.applied(success=False).outcome is MoveItPlanOutcome.FAILURE
    assert rig.node.events == [READ, APPLY]
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING


@pytest.mark.parametrize("unavailable", [READ, APPLY, PLAN])
def test_unavailable_service_never_falls_through_to_unchecked_planning(rig, unavailable):
    rig.node.clients[unavailable].available = False
    update = rig.planner.start(rig.target)
    if unavailable in (APPLY, PLAN):
        update = rig.read()
    if unavailable == PLAN:
        update = rig.applied()
    assert update.outcome is MoveItPlanOutcome.SERVICE_UNAVAILABLE
    assert not rig.node.clients[unavailable].requests
    assert not rig.node.clients[PLAN].requests
    assert not rig.planner.active
