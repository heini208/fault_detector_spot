"""Tests for guarded Cartesian close-surface motion selection."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.model.models import (
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
from fault_detector_spot.manipulation.behaviours.move_close_to_surface_behaviour import (
    MAX_CARTESIAN_CONTACT_MOVES,
    MAX_CARTESIAN_STANDOFF_MOVES,
    MoveCloseToSurfaceBehaviour,
    MoveCloseToSurfaceConfig,
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

    def start(self, target):
        self.normal_targets.append(target)
        return MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "normal")

    def start_cartesian(self, target):
        self.cartesian_targets.append(target)
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
    executor._moveit_cartesian_plan = None
    return executor, planner, plan


def test_executor_selects_cartesian_moveit_for_requested_probe_path():
    executor, planner, plan = executor_for_planning(True)

    update = executor._advance_moveit_planning_start()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(planner.cartesian_targets) == 1
    assert planner.normal_targets == []
    assert executor._moveit_cartesian_plan is plan
    assert not executor._pending_moveit_cartesian_path


def test_executor_keeps_normal_moveit_for_other_probe_paths():
    executor, planner, plan = executor_for_planning(False)

    update = executor._advance_moveit_planning_start()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(planner.normal_targets) == 1
    assert planner.cartesian_targets == []
    assert executor._moveit_cartesian_plan is plan


def test_cartesian_selection_is_consumed_by_primary_motion_only():
    executor = object.__new__(ArmMovementExecutor)
    executor._next_probe_cartesian_path = True
    executor._probe_continuation = object()
    captured = []

    def probe(*args, **kwargs):
        captured.append(kwargs)
        return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "running")

    executor.probe = probe

    executor._continue_probe(PoseStamped(), "sensor")
    executor._continue_probe(PoseStamped(), "sensor")

    assert captured[0]["_cartesian_path"] is True
    assert captured[1]["_cartesian_path"] is False


def action(contact=False):
    result = object.__new__(MoveCloseToSurfaceBehaviour)
    result.config = MoveCloseToSurfaceConfig()
    result._command = SimpleNamespace(
        target_surface_distance_m=0.0 if contact else 0.03
    )
    result._approach_steps = 0
    return result


def test_standoff_request_uses_full_remaining_distance():
    behaviour = action(contact=False)
    evaluation = SimpleNamespace(
        traveled_inward_m=0.04,
        remaining_inward_travel_m=0.12,
        requested_step_m=0.01,
    )

    requested = behaviour._requested_step(evaluation)

    assert requested == pytest.approx(0.12)
    assert requested > behaviour.config.maximum_step_m


def test_contact_request_adds_search_overtravel_once():
    behaviour = action(contact=True)
    evaluation = SimpleNamespace(
        traveled_inward_m=0.04,
        remaining_inward_travel_m=0.02,
        requested_step_m=0.01,
    )

    first = behaviour._requested_step(evaluation)
    behaviour._approach_steps = 1
    second = behaviour._requested_step(evaluation)

    assert first == pytest.approx(0.025)
    assert second == pytest.approx(0.0)


def test_cartesian_move_limits_allow_one_standoff_correction_only():
    standoff = action(contact=False)
    contact = action(contact=True)

    assert standoff._cartesian_movement_limit() == MAX_CARTESIAN_STANDOFF_MOVES
    assert contact._cartesian_movement_limit() == MAX_CARTESIAN_CONTACT_MOVES


def test_close_surface_requests_guarded_cartesian_execution(monkeypatch):
    behaviour = action(contact=False)
    behaviour._sensor_id = "probe"
    behaviour._attachment_revision = 1
    behaviour._plan = FrozenPlan()
    behaviour._aligned_probe_orientation = QuaternionData.identity()
    behaviour._previous_probe_pose = pose()
    behaviour._requested_step_m = 0.0
    behaviour._recovery_hand_pose = pose()
    behaviour._require_attachment_unchanged = lambda: None
    behaviour._current_probe_pose = lambda: pose()
    behaviour._validate_axis_guard = lambda evaluation: None

    captured = {}

    class FakeExecutor:
        active = False
        speed_policy = SimpleNamespace(
            default_speed=SimpleNamespace(angular_speed_rad_s=0.5)
        )

        def guarded_probe(self, *args, **kwargs):
            captured.update(kwargs)
            return ArmMovementUpdate(ArmMovementOutcome.RUNNING, "running")

    behaviour.executor = FakeExecutor()

    monkeypatch.setattr(
        "fault_detector_spot.manipulation.behaviours."
        "move_close_to_surface_behaviour.evaluate_probe_surface_approach",
        lambda *args, **kwargs: SimpleNamespace(
            reached=False,
            estimated_distance_m=0.15,
            remaining_inward_travel_m=0.12,
            traveled_inward_m=0.0,
            requested_step_m=0.01,
            axis_error_rad=0.0,
        ),
    )

    behaviour._prepare_next_approach_step()

    assert captured["cartesian_path"] is True
    assert behaviour._requested_step_m == pytest.approx(0.12)
