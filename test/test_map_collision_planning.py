"""Exercise real planner sequencing/conversion without ROS nodes or robot commands."""

from array import array
from copy import deepcopy
from dataclasses import dataclass
import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from octomap_msgs.msg import Octomap
from rclpy.time import Time
from rclpy.task import Future
from std_srvs.srv import Empty
import pytest

from fault_detector_spot.manipulation.moveit_arm_planner import (
    ARM_JOINT_NAMES, MoveItArmPlanner, MoveItPlanOutcome,
)
from fault_detector_spot.manipulation.rtabmap_collision_scene import RtabmapCollisionScene
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass(frozen=True)
class Session:
    started_at_ns: int = 9_000_000_000
    generation: int = 1


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
        self.requests.append(request)
        self.events.append(self.name)
        self.future = Future()
        return self.future


class Node:
    def __init__(self):
        self.clients = {}
        self.events = []
        self.now_ns = 10_000_000_000
        self.messages = []

    def get_logger(self):
        return SimpleNamespace(info=self.messages.append)

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.now_ns))

    def create_client(self, _type, name):
        client = Client(name, self.events)
        self.clients[name] = client
        return client

    def destroy_client(self, client):
        del self.clients[client.name]


class Rig:
    def __init__(self, enabled=True):
        self.node = Node()
        self.session = Session()
        self.tf = TransformStamped()
        self.tf.header.frame_id = "body"
        self.tf.child_frame_id = "map"
        self.tf.header.stamp.sec = 10
        self.tf.transform.rotation.w = 1.0
        self.tf.transform.translation.x = 0.4
        self.tf_reads = 0
        self.clock = 0.0
        runtime = SimpleNamespace(current_collision_map_session=lambda: self.session)
        self.scene = RtabmapCollisionScene(self.node, self, runtime) if enabled else None
        self.planner = MoveItArmPlanner(
            self.node, collision_scene=self.scene, monotonic_clock=lambda: self.clock,
        )
        self.target = PoseStamped()
        self.target.header.frame_id = "body"
        self.target.pose.position.x = 0.65
        self.target.pose.orientation.w = 1.0

    def lookup_transform(self, target, source, when):
        assert (target, source) == ("body", "map")
        self.tf_reads += 1
        return deepcopy(self.tf)

    def reply(self, name, response):
        self.node.clients[name].future.set_result(response)
        return self.planner.poll()

    def map_reply(self):
        snapshot = Octomap()
        snapshot.header.frame_id = "map"
        snapshot.binary = True
        snapshot.id = "ColorOcTree"
        snapshot.resolution = 0.05
        snapshot.data = array("b", [2, 0])  # Occupied root child.
        return self.reply("/rtabmap/octomap_binary", SimpleNamespace(map=snapshot))

    def applied(self, success=True):
        return self.reply("/apply_planning_scene", ApplyPlanningScene.Response(success=success))

    def planned(self):
        response = GetMotionPlan.Response()
        response.motion_plan_response.error_code.val = 1
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint(positions=[0.0] * 6)
        point.time_from_start.sec = 1
        trajectory.points = [point]
        return self.reply("/plan_kinematic_path", response)


def test_checked_bypass_checked_orders_scene_ack_before_original_plan():
    rig = Rig()
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == ["/rtabmap/octomap_binary"]
    rig.map_reply()
    assert rig.node.events[-1] == "/apply_planning_scene"
    assert not rig.node.clients["/plan_kinematic_path"].requests
    diff = rig.node.clients["/apply_planning_scene"].requests[0].scene
    assert diff.is_diff and diff.robot_state.is_diff
    assert diff.world.octomap.origin.position.x == pytest.approx(0.4)
    rig.applied()
    checked_request = rig.node.clients["/plan_kinematic_path"].requests[-1]
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS

    rig.session = None  # Bypass must remain usable after mapping stops.
    reads = rig.tf_reads
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert rig.node.events[-1] == "/clear_octomap"
    assert len(rig.node.clients["/plan_kinematic_path"].requests) == 1
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.node.clients["/plan_kinematic_path"].requests[-1] == checked_request
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.tf_reads == reads

    rig.session = Session(generation=2)
    rig.planner.start(rig.target)
    rig.map_reply()
    rig.applied()
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [
        "/rtabmap/octomap_binary", "/apply_planning_scene", "/plan_kinematic_path",
        "/clear_octomap", "/plan_kinematic_path",
        "/rtabmap/octomap_binary", "/apply_planning_scene", "/plan_kinematic_path",
    ]


def test_disabled_mode_uses_only_existing_planning_clients_and_same_request():
    rig = Rig(enabled=False)
    assert set(rig.node.clients) == {"/plan_kinematic_path", "/compute_cartesian_path"}
    expected = rig.planner._build_request(rig.target)
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert rig.node.events == ["/plan_kinematic_path"]
    assert rig.node.clients["/plan_kinematic_path"].requests[-1] == expected
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.tf_reads == 0


def test_cartesian_bypass_still_checks_self_collision_and_explicit_objects():
    rig = Rig()
    rig.planner.start_cartesian(rig.target, ignore_environment_collisions=True)
    rig.reply("/clear_octomap", Empty.Response())
    request = rig.node.clients["/compute_cartesian_path"].requests[-1]
    assert request == rig.planner._build_cartesian_request(rig.target)
    assert request.avoid_collisions


@pytest.mark.parametrize("change", ["session", "stopped", "translation", "rotation", "stale"])
@pytest.mark.parametrize("stage", ["map", "apply", "plan", "dispatch"])
def test_changed_map_or_placement_cannot_produce_or_dispatch_a_plan(change, stage):
    rig = Rig()
    rig.planner.start(rig.target)
    if stage in ("apply", "plan", "dispatch"):
        rig.map_reply()
    if stage in ("plan", "dispatch"):
        rig.applied()
    if stage == "dispatch":
        assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    if change == "session":
        rig.session = Session(generation=2)
    elif change == "stopped":
        rig.session = None
    elif change == "translation":
        rig.tf.transform.translation.z += 0.021
    elif change == "rotation":
        rig.tf.transform.rotation.z = 0.02
        rig.tf.transform.rotation.w = (1.0 - 0.02 ** 2) ** 0.5
    else:
        rig.node.now_ns += 2_000_000_000
    if stage == "dispatch":
        assert rig.planner.validate_prepared_scene() is not None
    else:
        update = {"map": rig.map_reply, "apply": rig.applied, "plan": rig.planned}[stage]()
        assert update.outcome is MoveItPlanOutcome.FAILURE
        assert update.trajectory is None


@pytest.mark.parametrize("problem", ["missing", "pre_session_tf", "stale_tf", "future_tf"])
def test_unavailable_placement_fails_before_mutation(problem):
    rig = Rig()
    if problem == "missing":
        rig.session = None
    elif problem == "pre_session_tf":
        rig.session = Session(started_at_ns=10_000_000_001)
    elif problem == "stale_tf":
        rig.tf.header.stamp.sec = 8
    else:
        rig.tf.header.stamp.sec = 11
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.events


def test_rejected_apply_never_submits_a_plan():
    rig = Rig()
    rig.planner.start(rig.target)
    rig.map_reply()
    assert rig.applied(success=False).outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.clients["/plan_kinematic_path"].requests


@pytest.mark.parametrize("phase", ["apply", "clear", "plan"])
@pytest.mark.parametrize("cancel", [False, True])
def test_timeout_or_cancel_waits_for_actual_server_response_before_next_policy(phase, cancel):
    rig = Rig()
    rig.planner.start(rig.target, ignore_environment_collisions=phase == "clear")
    if phase in ("apply", "plan"):
        rig.map_reply()
    if phase == "plan":
        rig.applied()
    future = rig.planner._future
    events = list(rig.node.events)
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 8.0
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    assert not future.cancelled()
    assert not rig.planner.active
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.ERROR
    assert "still running" in update.detail
    assert rig.node.events == events
    future.set_result(Empty.Response())  # Discard obsolete response, never consume its plan.
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == events + ["/clear_octomap"]


def test_failed_future_locks_out_further_mutations_instead_of_assuming_completion():
    rig = Rig()
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    rig.planner._future.set_exception(RuntimeError("transport lost"))
    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.ERROR
    assert rig.node.events == ["/clear_octomap"]


def test_unavailable_clear_does_not_fall_through_to_bypassed_planning():
    rig = Rig()
    rig.node.clients["/clear_octomap"].available = False
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.SERVICE_UNAVAILABLE
    assert not rig.node.events


def test_small_body_rotation_is_not_mistaken_for_translation_at_distant_map_origin():
    rig = Rig()
    rig.tf.transform.translation.x = -10.0
    rig.planner.start(rig.target)
    angle = 0.01
    rig.tf.transform.translation.x = -10.0 * math.cos(angle)
    rig.tf.transform.translation.y = 10.0 * math.sin(angle)
    rig.tf.transform.rotation.z = -math.sin(angle / 2)
    rig.tf.transform.rotation.w = math.cos(angle / 2)
    assert rig.planner.validate_prepared_scene() is None
    rig.map_reply()
    rig.applied()
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS


def test_malformed_snapshot_never_reaches_moveit():
    rig = Rig()
    rig.planner.start(rig.target)
    snapshot = Octomap(binary=False, id="ColorOcTree", resolution=0.05)
    update = rig.reply("/rtabmap/octomap_binary", SimpleNamespace(map=snapshot))
    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert rig.node.events == ["/rtabmap/octomap_binary"]


@pytest.mark.parametrize("cancel", [False, True])
def test_abandoned_map_read_cannot_block_bypass_or_import_its_late_response(cancel):
    rig = Rig()
    rig.planner.start(rig.target)
    old_future = rig.planner._future
    rig.session = None
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 8.0
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    old_future.set_result(SimpleNamespace(map=Octomap()))
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert "/apply_planning_scene" not in rig.node.events


@pytest.mark.parametrize("response_error", [False, True])
def test_cancel_inspects_already_done_uncertain_response_before_allowing_new_request(response_error):
    rig = Rig()
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    if response_error:
        rig.planner._future.set_exception(RuntimeError("transport lost"))
    else:
        rig.planner._future.set_result(None)
    rig.planner.cancel()
    update = rig.planner.start(rig.target, ignore_environment_collisions=True)
    assert update.outcome is MoveItPlanOutcome.ERROR
    assert "completion is unknown" in update.detail
    assert rig.node.events == ["/clear_octomap"]
