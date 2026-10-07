"""Collision intent survives command expansion and executor continuations."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand, SemanticTag, StampedPose,
)
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.manipulation.moveit_arm_planner import MoveItPlanOutcome, MoveItPlanUpdate


@pytest.mark.parametrize("ignore", [False, True])
def test_path_expansion_preserves_explicit_collision_intent(ignore):
    subscriber = CommandSubscriber()
    subscriber.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
    )
    command = SemanticCommand(
        command_id=CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        tag=SemanticTag(id=1, pose=StampedPose(frame_id="body")),
        motion_sensor_id="probe",
        pre_approach_offsets=(StampedPose(frame_id="body"),),
        ignore_environment_collisions=ignore,
    )
    translated = subscriber.fire_command_sequence(command)
    assert len(translated) == 2
    assert all(step.ignore_environment_collisions is ignore for step in translated)


@pytest.mark.parametrize("cartesian", [False, True])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("ignore", [False, True])
def test_planner_dispatch_preserves_policy_and_legacy_call_shape(cartesian, enabled, ignore):
    target = PoseStamped()
    plan = SimpleNamespace(target_hand=target, duration_sec=1.0)
    result = MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "planning")
    planner = SimpleNamespace(
        planning_frame="body",
        environment_collision_policy_enabled=enabled,
        start=Mock(return_value=result),
        start_cartesian=Mock(return_value=result),
    )
    executor = object.__new__(ArmMovementExecutor)
    executor.moveit_arm_planner = planner
    executor.probe_motion_planner = SimpleNamespace(normalize_target=lambda value, frame: value)
    executor._pending_moveit_plan_builder = lambda: plan
    executor._pending_moveit_cartesian_path = cartesian
    executor._ignore_environment_collisions = ignore

    executor._advance_moveit_planning_start()

    selected = planner.start_cartesian if cartesian else planner.start
    other = planner.start if cartesian else planner.start_cartesian
    options = {"ignore_environment_collisions": ignore} if enabled else {}
    selected.assert_called_once_with(target, **options)
    other.assert_not_called()
    assert executor._moveit_cartesian_plan is plan


@pytest.mark.parametrize("ignore", [False, True])
def test_corrections_retain_collision_policy_until_operation_reset(ignore):
    executor = object.__new__(ArmMovementExecutor)
    executor._clear_arm_operation_state()
    executor._ignore_environment_collisions = ignore
    executor._probe_continuation = object()
    executor.probe = Mock()
    for _ in range(2):
        executor._continue_probe(PoseStamped())
        assert executor.probe.call_args.kwargs["ignore_environment_collisions"] is ignore
    executor._clear_arm_operation_state()
    assert executor._ignore_environment_collisions is False


def test_contact_retreat_explicitly_bypasses_occupancy_only():
    executor = object.__new__(ArmMovementExecutor)
    executor._ignore_environment_collisions = False
    executor._probe_continuation = object()
    executor.probe = Mock()
    plan = object()
    executor._continue_contact_retreat(plan)
    executor.probe.assert_called_once_with(
        plan, ignore_environment_collisions=True,
        _continuation=executor._probe_continuation,
    )
