"""Tests for guarded Cartesian close-surface motion selection."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MAX_CARTESIAN_CONTACT_MOVES,
    MAX_CARTESIAN_STANDOFF_MOVES,
    MoveCloseToSurfaceConfig,
    MoveCloseToSurfaceExecution,
)
from fault_detector_spot.manipulation.moveit_arm_planner import (
    MoveItPlanOutcome,
    MoveItPlanUpdate,
)


def pose(x=0.0, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


class FakeMoveItPlanner:
    planning_frame = "body"

    def __init__(self):
        self.normal_targets = []
        self.cartesian_targets = []
        self.collision_options = []

    def start(self, target, *, ignore_environment_collisions=False):
        self.normal_targets.append(target)
        self.collision_options.append(ignore_environment_collisions)
        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "normal")

    def start_cartesian(self, target, *, ignore_environment_collisions=False):
        self.cartesian_targets.append(target)
        self.collision_options.append(ignore_environment_collisions)
        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "cartesian")


class FrozenPlan:
    @staticmethod
    def inward_direction():
        return Vector3Data(x=1.0, y=0.0, z=0.0)


def executor_for_planning(cartesian):
    executor = object.__new__(ArmMovementExecutor)
    target = PoseStamped()
    target.header.frame_id = "body"
    plan = SimpleNamespace(target_hand=target, duration_sec=1.0)
    planner = FakeMoveItPlanner()
    executor.moveit_arm_planner = planner
    executor.probe_motion_planner = SimpleNamespace(
        normalize_target=lambda value, frame: value
    )
    executor._pending_moveit_plan_builder = lambda: plan
    executor._pending_moveit_cartesian_path = cartesian
    executor._ignore_environment_collisions = False
    executor._moveit_cartesian_plan = None
    return executor, planner, plan


@pytest.mark.parametrize("bypass", [False, True])
def test_executor_selects_cartesian_moveit_for_requested_probe_path(bypass):
    executor, planner, plan = executor_for_planning(True)
    executor._ignore_environment_collisions = bypass

    update = executor._advance_moveit_planning_start()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(planner.cartesian_targets) == 1
    assert planner.normal_targets == []
    assert executor._moveit_cartesian_plan is plan
    assert not executor._pending_moveit_cartesian_path
    assert planner.collision_options == [bypass]


@pytest.mark.parametrize("bypass", [False, True])
def test_executor_keeps_normal_moveit_for_other_probe_paths(bypass):
    executor, planner, plan = executor_for_planning(False)
    executor._ignore_environment_collisions = bypass

    update = executor._advance_moveit_planning_start()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(planner.normal_targets) == 1
    assert planner.cartesian_targets == []
    assert executor._moveit_cartesian_plan is plan
    assert planner.collision_options == [bypass]


@pytest.mark.parametrize("bypass", [False, True])
def test_cartesian_selection_is_consumed_by_primary_motion_only(bypass):
    executor = object.__new__(ArmMovementExecutor)
    executor._next_probe_cartesian_path = True
    executor._ignore_environment_collisions = bypass
    executor._probe_continuation = object()
    captured = []

    def probe(*args, **kwargs):
        captured.append(kwargs)
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "running")

    executor.probe = probe

    executor._continue_probe(PoseStamped(), "sensor")
    executor._continue_probe(PoseStamped(), "sensor")
    executor._continue_contact_retreat(PoseStamped())

    assert captured[0]["_cartesian_path"] is True
    assert captured[1]["_cartesian_path"] is False
    assert [options["ignore_environment_collisions"] for options in captured] == [
        bypass, bypass, True,
    ]


def action(contact=False):
    result = object.__new__(MoveCloseToSurfaceExecution)
    result.config = MoveCloseToSurfaceConfig()
    result._command = SimpleNamespace(
        target_surface_distance_m=0.0 if contact else 0.03,
        ignore_environment_collisions=False,
    )
    result._approach_steps = 0
    return result


def test_standoff_request_uses_full_remaining_distance():
    execution = action(contact=False)
    evaluation = SimpleNamespace(
        traveled_inward_m=0.04,
        remaining_inward_travel_m=0.12,
        requested_step_m=0.01,
    )

    requested = execution._requested_step(evaluation)

    assert requested == pytest.approx(0.12)
    assert requested > execution.config.maximum_step_m


def test_contact_request_adds_search_overtravel_once():
    execution = action(contact=True)
    evaluation = SimpleNamespace(
        traveled_inward_m=0.04,
        remaining_inward_travel_m=0.02,
        requested_step_m=0.01,
    )

    first = execution._requested_step(evaluation)
    execution._approach_steps = 1
    second = execution._requested_step(evaluation)

    assert first == pytest.approx(0.025)
    assert second == pytest.approx(0.0)


def test_cartesian_move_limits_allow_one_standoff_correction_only():
    standoff = action(contact=False)
    contact = action(contact=True)

    assert standoff._cartesian_movement_limit() == MAX_CARTESIAN_STANDOFF_MOVES
    assert contact._cartesian_movement_limit() == MAX_CARTESIAN_CONTACT_MOVES


@pytest.mark.parametrize("bypass", [False, True])
def test_close_surface_requests_guarded_cartesian_execution(monkeypatch, bypass):
    execution = action(contact=False)
    execution._command.ignore_environment_collisions = bypass
    execution._sensor_id = "probe"
    execution._attachment_revision = 1
    execution._plan = FrozenPlan()
    execution._aligned_probe_orientation = QuaternionData.identity()
    execution._previous_probe_pose = pose()
    execution._requested_step_m = 0.0
    execution._recovery_hand_pose = pose()
    execution._require_attachment_unchanged = lambda: None
    execution._current_probe_pose = lambda: pose()
    execution._validate_axis_guard = lambda evaluation: None

    captured = {}

    class FakeExecutor:
        active = False
        speed_policy = SimpleNamespace(
            default_speed=SimpleNamespace(angular_speed_rad_s=0.5)
        )

        def guarded_probe(self, *args, **kwargs):
            captured.update(kwargs)
            return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "running")

    execution.executor = FakeExecutor()

    monkeypatch.setattr(
        "fault_detector_spot.manipulation."
        "move_close_to_surface_execution.evaluate_probe_surface_approach",
        lambda *args, **kwargs: SimpleNamespace(
            reached=False,
            estimated_distance_m=0.15,
            remaining_inward_travel_m=0.12,
            traveled_inward_m=0.0,
            requested_step_m=0.01,
            axis_error_rad=0.0,
        ),
    )

    execution._prepare_next_approach_step()

    assert captured["cartesian_path"] is True
    assert captured["ignore_environment_collisions"] is bypass
    assert execution._requested_step_m == pytest.approx(0.12)
