"""Idle scene updates share the planner's existing service sequence safely."""

from copy import deepcopy

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest

from fault_detector_spot.manipulation.moveit_arm_planner import MoveItArmPlanner, MoveItPlanOutcome
from fault_detector_spot.manipulation.moveit_collision_policy import OCTOMAP_OBJECT_ID
from fault_detector_spot.manipulation.sensor_head_geometry import SENSOR_HEAD_OBJECT_ID

from test_moveit_collision_planning import APPLY, CARTESIAN, PLAN, READ, Node
from test_moveit_sensor_head import mount, robot_description
from test_moveit_sensor_head_planning import ATTACHMENT, MESH, MODEL, head_rig


def advance_sync_to(rig, phase):
    update = rig.planner.start_scene_sync()
    while rig.planner._planning_mode != phase:
        assert update.outcome is MoveItPlanOutcome.RUNNING
        assert rig.planner.scene_sync_active
        update = rig.step()
    return update


def finish_sync(rig):
    while rig.planner.active:
        update = rig.step()
    return update


def assert_no_plan(rig):
    assert not rig.node.clients[PLAN].requests
    assert not rig.node.clients[CARTESIAN].requests


def test_scene_sync_applies_and_verifies_without_requesting_a_plan(head_rig):
    rig = head_rig
    assert not rig.planner.scene_sync_active
    advance_sync_to(rig, "head_verify")
    assert rig.planner.scene_sync_active
    assert_no_plan(rig)

    update = rig.attachment()

    assert update.outcome is MoveItPlanOutcome.SUCCESS
    assert update.trajectory is None
    assert not rig.planner.active
    assert not rig.planner.scene_sync_active
    assert_no_plan(rig)
    assert rig.node.events == [ATTACHMENT, MODEL, MESH, MESH, READ, APPLY, READ, ATTACHMENT]


@pytest.mark.parametrize("cartesian", [False, True])
@pytest.mark.parametrize("bypass", [False, True])
def test_command_takes_over_sync_then_runs_its_own_complete_preflight(head_rig, cartesian, bypass):
    rig = head_rig
    rig.toggle()
    advance_sync_to(rig, "apply")
    in_flight = rig.planner._future
    events = list(rig.node.events)
    start = rig.planner.start_cartesian if cartesian else rig.planner.start
    expected = (rig.planner._build_cartesian_request(rig.target) if cartesian
                else rig.planner._build_request(rig.target))

    update = start(rig.target, ignore_environment_collisions=bypass)
    rig.target.pose.position.x += .1

    assert update.outcome is MoveItPlanOutcome.RUNNING
    assert not rig.planner.scene_sync_active
    assert rig.planner.active
    assert rig.planner._future is in_flight
    assert rig.node.events == events
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.ERROR
    assert rig.applied().outcome is MoveItPlanOutcome.RUNNING
    assert rig.read().outcome is MoveItPlanOutcome.RUNNING
    assert rig.attachment().outcome is MoveItPlanOutcome.RUNNING
    assert rig.planner._planning_mode == "head_attachment"
    assert not rig.planner.scene_sync_active
    assert_no_plan(rig)

    mode = "cartesian" if cartesian else "motion"
    while rig.planner._planning_mode != mode:
        assert rig.step().outcome is MoveItPlanOutcome.RUNNING
    service = CARTESIAN if cartesian else PLAN
    assert rig.node.clients[service].requests == [expected]
    matrices = [request.scene.allowed_collision_matrix for request in rig.node.clients[APPLY].requests]
    for matrix, ignored in zip(matrices, (False, bypass), strict=True):
        row = matrix.entry_values[matrix.entry_names.index(SENSOR_HEAD_OBJECT_ID)].enabled
        assert row[matrix.entry_names.index(OCTOMAP_OBJECT_ID)] is ignored
    assert rig.finish_plan().outcome is MoveItPlanOutcome.SUCCESS
    assert rig.node.events == [
        ATTACHMENT, MODEL, MESH, MESH, READ, APPLY, READ, ATTACHMENT,
        ATTACHMENT, MODEL, READ, APPLY, READ, ATTACHMENT, service, READ, ATTACHMENT,
    ]


@pytest.mark.parametrize("invalid", ["type", "frame", "bypass"])
def test_invalid_command_does_not_take_polling_ownership(head_rig, invalid):
    rig = head_rig
    advance_sync_to(rig, "apply")
    target = deepcopy(rig.target)
    if invalid == "type":
        target = None
    elif invalid == "frame":
        target.header.frame_id = "incorrect"

    update = rig.planner.start(target, ignore_environment_collisions=1 if invalid == "bypass" else False)

    assert update.outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.scene_sync_active
    assert finish_sync(rig).outcome is MoveItPlanOutcome.SUCCESS
    assert_no_plan(rig)


@pytest.mark.parametrize("damage", ["apply_rejected", "missing_geometry", "revision"])
def test_failed_sync_discards_queued_command(head_rig, damage):
    rig = head_rig
    advance_sync_to(rig, "apply")
    rig.planner.start(rig.target)
    if damage == "apply_rejected":
        update = rig.applied(success=False)
    else:
        rig.applied()
        if damage == "missing_geometry":
            rig.server_scene.robot_state.attached_collision_objects = []
            update = rig.read()
        else:
            rig.read()
            rig.registry.state.attachment_revision += 1
            update = rig.attachment()

    assert update.outcome is MoveItPlanOutcome.FAILURE
    assert update.trajectory is None
    assert not rig.planner.active
    assert not rig.planner.scene_sync_active
    assert_no_plan(rig)
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.RUNNING
    assert finish_sync(rig).outcome is MoveItPlanOutcome.SUCCESS
    assert_no_plan(rig)


@pytest.mark.parametrize("timeout", [False, True])
def test_cancelled_sync_waits_for_late_apply_and_discards_queued_command(head_rig, timeout):
    rig = head_rig
    advance_sync_to(rig, "apply")
    previous = rig.planner._future
    rig.planner.start(rig.target)
    events = list(rig.node.events)
    if timeout:
        rig.clock += rig.planner.response_timeout_sec
        assert rig.planner.poll().outcome is MoveItPlanOutcome.TIMEOUT
    else:
        rig.planner.cancel()

    assert not rig.planner.active
    assert not rig.planner.scene_sync_active
    assert not previous.cancelled()
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.ERROR
    assert rig.planner.start(rig.target).outcome is MoveItPlanOutcome.ERROR
    assert rig.node.events == events
    previous.set_result(ApplyPlanningScene.Response(success=True))
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.RUNNING
    assert finish_sync(rig).outcome is MoveItPlanOutcome.SUCCESS
    assert_no_plan(rig)


def test_cancelled_read_cannot_later_apply_or_release_queued_command(head_rig):
    rig = head_rig
    advance_sync_to(rig, "read_scene")
    previous = rig.planner._future
    rig.planner.start_cartesian(rig.target)
    rig.planner.cancel()
    previous.set_result(GetPlanningScene.Response(scene=deepcopy(rig.server_scene)))

    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert not rig.node.clients[APPLY].requests
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.RUNNING
    assert finish_sync(rig).outcome is MoveItPlanOutcome.SUCCESS
    assert_no_plan(rig)


def test_unknown_apply_completion_blocks_sync_and_plans(head_rig):
    rig = head_rig
    advance_sync_to(rig, "apply")
    rig.planner.start(rig.target)
    rig.planner._future.set_exception(RuntimeError("connection lost"))

    assert rig.planner.poll().outcome is MoveItPlanOutcome.ERROR
    assert "completion is unknown" in rig.planner.start_scene_sync().detail
    assert "completion is unknown" in rig.planner.start(rig.target).detail
    assert not rig.planner.scene_sync_active
    assert_no_plan(rig)


def test_scene_sync_cannot_interrupt_a_plan_or_another_sync(head_rig):
    rig = head_rig
    advance_sync_to(rig, "apply")
    events = list(rig.node.events)
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.ERROR
    assert rig.node.events == events
    finish_sync(rig)
    rig.advance_to("motion")
    events = list(rig.node.events)
    assert rig.planner.start_scene_sync().outcome is MoveItPlanOutcome.ERROR
    assert rig.node.events == events


def test_scene_sync_requires_a_collision_scene():
    node = Node()
    planner = MoveItArmPlanner(node)
    try:
        assert planner.start_scene_sync().outcome is MoveItPlanOutcome.ERROR
        assert not planner.active
        assert not node.events
    finally:
        planner.destroy()
