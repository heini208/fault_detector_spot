"""Exercise head resolution, scene acknowledgement and planning as one sequence."""

from copy import deepcopy
from types import SimpleNamespace

from fault_detector_msgs.srv import GetSensorMesh
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, AttachedCollisionObject,
    CollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters

from fault_detector_spot.inspection.model.sensor_geometry import MAX_MESH_CHUNK_BYTES
from fault_detector_spot.manipulation.arm_collision_control import ArmCollisionPolicy
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItArmPlanner, MoveItPlanOutcome
from fault_detector_spot.manipulation.moveit_collision_policy import OCTOMAP_OBJECT_ID
from fault_detector_spot.manipulation.moveit_collision_scene import MoveItCollisionScene
from fault_detector_spot.manipulation.moveit_sensor_head import MoveItSensorHead
from fault_detector_spot.manipulation.sensor_head_geometry import SENSOR_HEAD_OBJECT_ID

from test_moveit_collision_planning import APPLY, CARTESIAN, PLAN, READ, Node, Rig
from test_moveit_sensor_head import attachment_response, mount, robot_description


ATTACHMENT = "fault_detector/application/get_sensor_attachment"
MODEL = "/move_group/get_parameters"
MESH = "fault_detector/get_sensor_mesh"


class HeadNode(Node):
    def create_subscription(self, _message_type, _topic, callback, _qos):
        return SimpleNamespace(callback=callback)

    def destroy_subscription(self, _subscription):
        pass


class HeadRig(Rig):
    def __init__(self, mount, robot_description):
        self.node = HeadNode()
        self.clock = 0.0
        self.policy = ArmCollisionPolicy(enabled=False, revision=0)
        self.head = MoveItSensorHead(self.node)
        self.scene = MoveItCollisionScene(
            self.node, SimpleNamespace(state=lambda: self.policy), sensor_head=self.head,
        )
        self.planner = MoveItArmPlanner(
            self.node, collision_scene=self.scene, monotonic_clock=lambda: self.clock,
        )
        self.target = PoseStamped()
        self.target.header.frame_id = "body"
        self.target.pose.position.x = .65
        self.target.pose.orientation.w = 1.
        self.registry = attachment_response(mount.geometry)
        self.mesh_bytes = mount.data
        self.robot_description = robot_description
        self.server_scene = PlanningScene()
        self.server_scene.allowed_collision_matrix = AllowedCollisionMatrix(
            entry_names=["arm", "body", "wall"],
            entry_values=[AllowedCollisionEntry(enabled=row) for row in (
                [True, False, False], [False, True, False], [False, False, True],
            )],
        )
        self.server_scene.world.collision_objects = [CollisionObject(id="wall")]
        self.server_scene.robot_state.attached_collision_objects = [
            AttachedCollisionObject(object=CollisionObject(id="other_attachment")),
        ]

    def attachment(self):
        return self.reply(ATTACHMENT, deepcopy(self.registry))

    def model(self):
        return self.reply(MODEL, GetParameters.Response(values=[ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=self.robot_description,
        )]))

    def mesh(self):
        request = self.node.clients[MESH].requests[-1]
        data = self.mesh_bytes[request.offset:request.offset + MAX_MESH_CHUNK_BYTES]
        return self.reply(MESH, GetSensorMesh.Response(
            success=True, complete=request.offset + len(data) == len(self.mesh_bytes), data=data,
        ))

    def read(self):
        return self.reply(READ, GetPlanningScene.Response(scene=deepcopy(self.server_scene)))

    def applied(self, success=True):
        if success:
            diff = self.node.clients[APPLY].requests[-1].scene
            self.server_scene.allowed_collision_matrix = deepcopy(diff.allowed_collision_matrix)
            for attached in diff.robot_state.attached_collision_objects:
                self.server_scene.robot_state.attached_collision_objects = [
                    obj for obj in self.server_scene.robot_state.attached_collision_objects
                    if obj.object.id != attached.object.id
                ] + [deepcopy(attached)]
            # MoveIt consumes any world copy when attaching the same object ID.
            removed = {obj.object.id for obj in diff.robot_state.attached_collision_objects}
            self.server_scene.world.collision_objects = [
                obj for obj in self.server_scene.world.collision_objects if obj.id not in removed
            ]
        return self.reply(APPLY, ApplyPlanningScene.Response(success=success))

    def advance_to(self, phase, *, bypass=False, cartesian=False):
        start = self.planner.start_cartesian if cartesian else self.planner.start
        update = start(self.target, ignore_environment_collisions=bypass)
        while self.planner._planning_mode != phase:
            assert update.outcome is MoveItPlanOutcome.RUNNING
            update = self.step()
        return update

    def step(self):
        return {
            "head_attachment": self.attachment, "head_model": self.model,
            "head_mesh": self.mesh, "read_scene": self.read,
            "apply": self.applied, "verify_scene": self.read,
            "head_verify": self.attachment,
        }[self.planner._planning_mode]()

    def finish_plan(self):
        assert self.planned().outcome is MoveItPlanOutcome.RUNNING
        assert self.read().outcome is MoveItPlanOutcome.RUNNING
        return self.attachment()


@pytest.fixture
def head_rig(mount, robot_description):
    rig = HeadRig(mount, robot_description)
    yield rig
    rig.planner.destroy()


@pytest.mark.parametrize("cartesian", [False, True])
def test_head_is_applied_and_verified_before_and_after_planning(head_rig, cartesian):
    rig = head_rig
    rig.server_scene.world.collision_objects.append(CollisionObject(id=SENSOR_HEAD_OBJECT_ID))
    mode = "cartesian" if cartesian else "motion"
    plan_service = CARTESIAN if cartesian else PLAN

    rig.advance_to("apply", cartesian=cartesian)

    assert not rig.node.clients[plan_service].requests
    diff = rig.node.clients[APPLY].requests[-1].scene
    assert diff.is_diff and diff.robot_state.is_diff
    assert [obj.object.id for obj in diff.robot_state.attached_collision_objects] == [SENSOR_HEAD_OBJECT_ID]
    assert not diff.world.collision_objects
    assert diff.object_colors[0].id == SENSOR_HEAD_OBJECT_ID
    assert diff.object_colors[0].color.a > 0
    rig.applied()
    assert not rig.node.clients[plan_service].requests
    rig.read()
    assert not rig.node.clients[plan_service].requests
    rig.attachment()
    assert rig.planner._planning_mode == mode
    assert rig.finish_plan().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [
        ATTACHMENT, MODEL, MESH, MESH, READ, APPLY, READ, ATTACHMENT,
        plan_service, READ, ATTACHMENT,
    ]
    assert [obj.id for obj in rig.server_scene.world.collision_objects] == ["wall"]
    assert [obj.object.id for obj in rig.server_scene.robot_state.attached_collision_objects] == [
        "other_attachment", SENSOR_HEAD_OBJECT_ID,
    ]


@pytest.mark.parametrize("enabled,bypass", [(False, False), (True, False), (True, True)])
def test_new_head_obeys_occupancy_toggle_but_keeps_robot_and_explicit_objects_checked(
    head_rig, enabled, bypass,
):
    rig = head_rig
    if enabled:
        rig.toggle()
    rig.advance_to("apply", bypass=bypass)
    matrix = rig.node.clients[APPLY].requests[-1].scene.allowed_collision_matrix
    row = matrix.entry_values[matrix.entry_names.index(SENSOR_HEAD_OBJECT_ID)].enabled

    assert row[matrix.entry_names.index(OCTOMAP_OBJECT_ID)] is (bypass or not enabled)
    assert not row[matrix.entry_names.index("body")]
    assert not row[matrix.entry_names.index("wall")]


def test_custom_stock_and_custom_swap_replace_the_same_owned_attachment(head_rig):
    rig = head_rig
    custom = deepcopy(rig.registry)
    for response, expected_mesh_count in (
        (custom, 1), (attachment_response(None, "", revision=4), 3), (custom, 1),
    ):
        rig.registry = response
        rig.advance_to("motion")
        assert rig.finish_plan().outcome is MoveItPlanOutcome.SUCCESS
        attached = [obj for obj in rig.server_scene.robot_state.attached_collision_objects
                    if obj.object.id == SENSOR_HEAD_OBJECT_ID]
        assert len(attached) == 1
        assert len(attached[0].object.meshes) == expected_mesh_count
        assert all(obj.id != SENSOR_HEAD_OBJECT_ID for obj in rig.server_scene.world.collision_objects)


@pytest.mark.parametrize("after_plan", [False, True])
@pytest.mark.parametrize("damage", ["missing", "mesh", "world_copy", "touch", "occupancy"])
def test_changed_scene_never_releases_a_trajectory(head_rig, after_plan, damage):
    rig = head_rig
    rig.advance_to("motion" if after_plan else "verify_scene")
    if after_plan:
        assert rig.planned().outcome is MoveItPlanOutcome.RUNNING
    attached = rig.server_scene.robot_state.attached_collision_objects
    head = next(obj for obj in attached if obj.object.id == SENSOR_HEAD_OBJECT_ID)
    if damage == "missing":
        attached.remove(head)
    elif damage == "mesh":
        head.object.meshes[0].vertices[0].x += .001
    elif damage == "world_copy":
        rig.server_scene.world.collision_objects.append(CollisionObject(id=SENSOR_HEAD_OBJECT_ID))
    elif damage == "touch":
        head.touch_links.append("body")
    else:
        matrix = rig.server_scene.allowed_collision_matrix
        i, j = (matrix.entry_names.index(name) for name in (SENSOR_HEAD_OBJECT_ID, OCTOMAP_OBJECT_ID))
        matrix.entry_values[i].enabled[j] = False
        matrix.entry_values[j].enabled[i] = False

    update = rig.read()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert update.trajectory is None
    assert len(rig.node.clients[PLAN].requests) == int(after_plan)


@pytest.mark.parametrize("after_plan", [False, True])
def test_registry_revision_change_in_verification_never_releases_a_trajectory(head_rig, after_plan):
    rig = head_rig
    rig.advance_to("motion" if after_plan else "head_verify")
    if after_plan:
        rig.planned()
        rig.read()
    rig.registry.state.attachment_revision += 1

    update = rig.attachment()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert update.trajectory is None
    assert "changed" in update.detail


def test_missing_geometry_fails_before_any_scene_mutation_or_plan(head_rig):
    rig = head_rig
    rig.registry.head_geometry = []
    rig.planner.start(rig.target)

    update = rig.attachment()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert "no collision geometry" in update.detail
    assert rig.node.events == [ATTACHMENT]


def test_readback_checks_every_stock_mesh(head_rig):
    rig = head_rig
    rig.registry = attachment_response(None, "", revision=4)
    rig.advance_to("verify_scene")
    head = next(obj for obj in rig.server_scene.robot_state.attached_collision_objects
                if obj.object.id == SENSOR_HEAD_OBJECT_ID)
    assert len(head.object.meshes) == 3
    head.object.meshes[-1].vertices[-1].z += .001

    update = rig.read()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.clients[PLAN].requests


def test_observed_attachment_change_invalidates_even_an_already_returned_plan(head_rig):
    rig = head_rig
    rig.advance_to("motion")
    assert rig.finish_plan().outcome is MoveItPlanOutcome.SUCCESS
    changed = deepcopy(rig.registry.state)
    changed.attachment_revision += 1
    rig.head._observe_state(changed)

    assert "changed" in rig.planner.validate_prepared_scene()


@pytest.mark.parametrize("service", [ATTACHMENT, MODEL, MESH, READ, APPLY, PLAN])
def test_unavailable_service_does_not_skip_head_checks(head_rig, service):
    rig = head_rig
    rig.node.clients[service].available = False
    update = rig.planner.start(rig.target)
    while update.outcome is MoveItPlanOutcome.RUNNING:
        update = rig.step()

    assert update.outcome is MoveItPlanOutcome.SERVICE_UNAVAILABLE
    assert update.trajectory is None
    assert not rig.node.clients[service].requests
    assert not rig.node.clients[PLAN].requests


def test_rejected_apply_does_not_plan_or_verify_as_if_geometry_existed(head_rig):
    rig = head_rig
    rig.advance_to("apply")

    update = rig.applied(success=False)

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert not rig.node.clients[PLAN].requests
    assert rig.node.events[-1] == APPLY


def test_cancelled_apply_waits_for_late_server_completion_before_new_head_update(head_rig):
    rig = head_rig
    rig.advance_to("apply")
    previous = rig.planner._future
    events = list(rig.node.events)

    rig.planner.cancel()
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.ERROR
    assert rig.node.events == events
    assert not previous.cancelled()
    previous.set_result(ApplyPlanningScene.Response(success=True))
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.node.events == events + [ATTACHMENT]
    assert not rig.node.clients[PLAN].requests


def test_cancel_during_final_read_discards_prepared_trajectory(head_rig):
    rig = head_rig
    rig.advance_to("motion")
    rig.planned()
    previous = rig.planner._future

    rig.planner.cancel()
    previous.set_result(GetPlanningScene.Response(scene=deepcopy(rig.server_scene)))

    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.RUNNING
    assert rig.planner._completed_plan is None
