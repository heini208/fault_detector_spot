"""Exercise real planner sequencing/conversion without ROS nodes or robot commands."""

from array import array
from concurrent.futures import wait
from copy import deepcopy
from dataclasses import dataclass
import math
from threading import Event, Thread
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
from fault_detector_spot.manipulation import rtabmap_collision_scene
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
        self.policy_enabled = True
        self.policy_revision = 0
        self.tf = TransformStamped()
        self.tf.header.frame_id = "body"
        self.tf.child_frame_id = "map"
        self.tf.header.stamp.sec = 10
        self.tf.transform.rotation.w = 1.0
        self.tf.transform.translation.x = 0.4
        self.tf_reads = 0
        self.clock = 0.0
        runtime = SimpleNamespace(
            current_collision_map_session=lambda: self.session,
            collision_checking_state=lambda: SimpleNamespace(
                session=self.session,
                enabled=self.policy_enabled and self.session is not None,
                revision=self.policy_revision,
            ),
        )
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
        update = self.planner.poll()
        if self.planner._planning_mode == "prepare_map":
            done, _ = wait([self.planner._future], timeout=2.0)
            assert done, "Local map conversion did not finish"
            update = self.planner.poll()
        return update

    @staticmethod
    def map_response():
        snapshot = Octomap()
        snapshot.header.frame_id = "map"
        snapshot.binary = True
        snapshot.id = "ColorOcTree"
        snapshot.resolution = 0.05
        snapshot.data = array("b", [2, 0])  # Occupied root child.
        return SimpleNamespace(map=snapshot)

    def map_reply(self):
        return self.reply("/rtabmap/octomap_binary", self.map_response())

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


@pytest.fixture
def converting_map(monkeypatch):
    rig = Rig()
    entered, release = Event(), Event()
    convert = rtabmap_collision_scene.snapshot_scene_diff
    future = None

    def blocked_conversion(*args):
        entered.set()
        assert release.wait(5.0), "Test did not release map conversion"
        return convert(*args)

    monkeypatch.setattr(rtabmap_collision_scene, "snapshot_scene_diff", blocked_conversion)
    try:
        rig.planner.start(rig.target)
        rig.planner._future.set_result(rig.map_response())
        assert rig.planner.poll().outcome is MoveItPlanOutcome.RUNNING
        future = rig.planner._future
        assert entered.wait(2.0)
        assert rig.planner._planning_mode == "prepare_map"
        yield SimpleNamespace(rig=rig, release=release, future=future)
    finally:
        release.set()
        if future is not None:
            wait([future], timeout=2.0)
        if rig.node.clients:
            rig.planner.destroy()


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


def test_planner_without_scene_adapter_keeps_original_request():
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


@pytest.mark.parametrize("problem", ["pre_session_tf", "stale_tf", "future_tf"])
def test_unavailable_placement_fails_before_mutation(problem):
    rig = Rig()
    if problem == "pre_session_tf":
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
        update = rig.planner.poll()
        assert update.outcome is MoveItPlanOutcome.TIMEOUT
        assert "7.0 s" in update.detail
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
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS


@pytest.mark.parametrize("cancel", [False, True])
def test_abandoned_map_read_cannot_block_bypass_or_import_its_late_response(cancel):
    rig = Rig()
    rig.planner.start(rig.target)
    old_future = rig.planner._future
    rig.session = None
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 21.0
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    old_future.set_result(SimpleNamespace(map=Octomap()))
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert "/apply_planning_scene" not in rig.node.events


def test_map_read_can_take_eleven_seconds_without_extending_other_phase_timeouts():
    rig = Rig()
    rig.planner.start(rig.target)
    rig.clock = 11.0
    assert rig.planner.poll().outcome is MoveItPlanOutcome.RUNNING
    rig.map_reply()
    rig.applied()
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS


@pytest.mark.parametrize("cancel", [False, True])
def test_abandoned_conversion_cannot_block_bypass_or_apply_its_late_result(converting_map, cancel):
    rig = converting_map.rig
    assert converting_map.future.running()
    if cancel:
        rig.planner.cancel()
    else:
        rig.clock = 8.0
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    assert rig.planner.start(rig.target, ignore_environment_collisions=True).outcome is MoveItPlanOutcome.RUNNING
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert not converting_map.future.done()
    converting_map.release.set()
    converting_map.future.result(timeout=2.0)
    assert not rig.planner.active
    assert rig.node.events == [
        "/rtabmap/octomap_binary", "/clear_octomap", "/plan_kinematic_path",
    ]


@pytest.mark.parametrize("change", ["policy", "session", "placement", "stale_tf"])
def test_conversion_revalidates_before_applying_its_result(converting_map, change):
    rig = converting_map.rig
    if change == "policy":
        rig.policy_enabled = False
        rig.policy_revision += 1
    elif change == "session":
        rig.session = Session(generation=2)
    elif change == "placement":
        rig.tf.transform.translation.z += 0.021
    else:
        rig.node.now_ns += 2_000_000_000
    assert rig.planner.poll().outcome is MoveItPlanOutcome.RUNNING
    converting_map.release.set()
    converting_map.future.result(timeout=2.0)
    assert rig.planner.poll().outcome is MoveItPlanOutcome.FAILURE
    assert rig.node.events == ["/rtabmap/octomap_binary"]


def test_destroy_does_not_wait_for_conversion_or_apply_its_result(converting_map):
    rig = converting_map.rig
    destroyed = Event()

    def destroy():
        try:
            rig.planner.destroy()
        finally:
            destroyed.set()

    worker = Thread(target=destroy)
    worker.start()
    try:
        assert destroyed.wait(1.0), "Destroy waited for read-only map conversion"
        assert not converting_map.future.done()
        assert not rig.node.clients
    finally:
        converting_map.release.set()
        worker.join(timeout=2.0)
    converting_map.future.result(timeout=2.0)
    assert not rig.planner.active
    assert rig.node.events == ["/rtabmap/octomap_binary"]


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


def test_no_map_plans_normally_after_acknowledged_clear_without_tf():
    rig = Rig()
    rig.session = None
    rig.tf.header.stamp.sec = 0
    expected = rig.planner._build_request(rig.target)
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == ["/clear_octomap"]
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.node.clients["/plan_kinematic_path"].requests[-1] == expected
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.tf_reads == 0


def test_mapping_auto_enable_manual_off_on_and_stop_select_correct_requests():
    rig = Rig()
    rig.session = None
    rig.planner.start(rig.target)
    rig.reply("/clear_octomap", Empty.Response())
    rig.planned()
    rig.session = Session()
    rig.planner.start(rig.target)
    rig.map_reply()
    rig.applied()
    rig.planned()
    rig.policy_enabled = False
    rig.policy_revision += 1
    # Global manual-off is persistent across ordinary commands.
    for _ in range(2):
        rig.planner.start(rig.target)
        rig.reply("/clear_octomap", Empty.Response())
        assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    rig.policy_enabled = True
    rig.policy_revision += 1
    rig.planner.start(rig.target)
    rig.map_reply()
    rig.applied()
    rig.planned()
    rig.session = None
    rig.planner.start(rig.target)
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [
        "/clear_octomap", "/plan_kinematic_path",
        "/rtabmap/octomap_binary", "/apply_planning_scene", "/plan_kinematic_path",
        "/clear_octomap", "/plan_kinematic_path",
        "/clear_octomap", "/plan_kinematic_path",
        "/rtabmap/octomap_binary", "/apply_planning_scene", "/plan_kinematic_path",
        "/clear_octomap", "/plan_kinematic_path",
    ]


@pytest.mark.parametrize("stage", ["map", "apply", "plan", "dispatch"])
def test_global_disable_invalidates_checked_work_before_dispatch(stage):
    rig = Rig()
    rig.planner.start(rig.target)
    if stage in ("apply", "plan", "dispatch"):
        rig.map_reply()
    if stage in ("plan", "dispatch"):
        rig.applied()
    if stage == "dispatch":
        assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    rig.policy_enabled = False
    rig.policy_revision += 1
    if stage == "dispatch":
        assert "setting changed" in rig.planner.validate_prepared_scene()
    else:
        update = {"map": rig.map_reply, "apply": rig.applied, "plan": rig.planned}[stage]()
        assert update.outcome is MoveItPlanOutcome.FAILURE
        assert update.trajectory is None


@pytest.mark.parametrize("change", ["map_started", "manual_on", "off_on_off"])
def test_new_checking_policy_invalidates_previously_unchecked_preparation(change):
    rig = Rig()
    rig.policy_enabled = change == "map_started"
    if change == "map_started":
        rig.session = None
    rig.planner.start(rig.target)
    if change == "map_started":
        rig.session = Session()
    elif change == "manual_on":
        rig.policy_enabled = True
        rig.policy_revision += 1
    else:
        rig.policy_revision += 2
    update = rig.reply("/clear_octomap", Empty.Response())
    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.clients["/plan_kinematic_path"].requests


def test_explicit_command_bypass_is_independent_of_global_policy_changes():
    rig = Rig()
    rig.planner.start(rig.target, ignore_environment_collisions=True)
    rig.policy_enabled = False
    rig.policy_revision += 1
    rig.reply("/clear_octomap", Empty.Response())
    assert rig.planned().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.tf_reads == 0


def test_global_off_still_waits_for_previous_uncertain_scene_mutation():
    rig = Rig()
    rig.planner.start(rig.target)
    rig.map_reply()
    old_apply = rig.planner._future
    rig.clock = 8.0
    assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    rig.policy_enabled = False
    rig.policy_revision += 1
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.ERROR
    old_apply.set_result(ApplyPlanningScene.Response(success=True))
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events[-1] == "/clear_octomap"
