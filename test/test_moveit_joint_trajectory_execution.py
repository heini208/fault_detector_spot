"""Tests for MoveIt joint-trajectory execution on Spot."""

from types import SimpleNamespace

import pytest

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.moveit_arm_planner import (
    DEFAULT_ACCELERATION_SCALING,
    DEFAULT_VELOCITY_SCALING,
    MoveItPlanOutcome,
    MoveItPlanUpdate,
)


MOVEIT_ORDER = (
    "arm_wr1",
    "arm_el0",
    "arm_sh0",
    "arm_wr0",
    "arm_sh1",
    "arm_el1",
)


def trajectory(with_velocities=True):
    velocity_a = [0.6, 0.3, 0.1, 0.5, 0.2, 0.4]
    velocity_b = [1.6, 1.3, 1.1, 1.5, 1.2, 1.4]
    if not with_velocities:
        velocity_a = []
        velocity_b = []
    return SimpleNamespace(
        joint_names=list(MOVEIT_ORDER),
        points=[
            SimpleNamespace(
                positions=[6.0, 3.0, 1.0, 5.0, 2.0, 4.0],
                velocities=velocity_a,
                time_from_start=SimpleNamespace(
                    sec=0,
                    nanosec=500_000_000,
                ),
            ),
            SimpleNamespace(
                positions=[16.0, 13.0, 11.0, 15.0, 12.0, 14.0],
                velocities=velocity_b,
                time_from_start=SimpleNamespace(
                    sec=1,
                    nanosec=250_000_000,
                ),
            ),
        ],
    )


def capture_joint_move(monkeypatch):
    captured = {}

    def build(joint_positions, times, joint_velocities=None, **kwargs):
        captured["joint_positions"] = joint_positions
        captured["times"] = times
        captured["joint_velocities"] = joint_velocities
        captured["kwargs"] = kwargs
        return object()

    monkeypatch.setattr(
        executor_module.RobotCommandBuilder,
        "arm_joint_move_helper",
        build,
    )
    monkeypatch.setattr(
        executor_module,
        "convert",
        lambda source, target: None,
    )
    return captured


def test_moveit_success_executes_returned_joint_trajectory(monkeypatch):
    planned = trajectory()
    executor = object.__new__(ArmMovementExecutor)
    executor.moveit_arm_planner = SimpleNamespace(
        poll=lambda: MoveItPlanUpdate(
            MoveItPlanOutcome.SUCCESS,
            "planned",
            trajectory=planned,
        )
    )
    executor._moveit_cartesian_plan = SimpleNamespace(duration_sec=1.0)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0
    submitted = {}

    def submit(goal_builder):
        submitted["goal_builder"] = goal_builder
        return ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "submitted",
        )

    executor._submit_goal = submit
    captured = capture_joint_move(monkeypatch)

    update = executor._poll_moveit_planning()

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert executor._moveit_cartesian_plan is None
    submitted["goal_builder"]()

    assert captured["joint_positions"] == [
        pytest.approx([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
        pytest.approx([11.0, 12.0, 13.0, 14.0, 15.0, 16.0]),
    ]
    assert captured["times"] == pytest.approx([0.5, 1.25])
    assert captured["joint_velocities"] == [
        pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
        pytest.approx([1.1, 1.2, 1.3, 1.4, 1.5, 1.6]),
    ]


def test_moveit_joint_goal_allows_position_only_trajectory(monkeypatch):
    captured = capture_joint_move(monkeypatch)

    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0

    executor._build_moveit_joint_goal(
        trajectory(with_velocities=False)
    )

    assert captured["joint_velocities"] is None


def test_moveit_joint_goal_rejects_partial_velocity_data(monkeypatch):
    planned = trajectory()
    planned.points[1].velocities = []
    capture_joint_move(monkeypatch)

    with pytest.raises(ValueError, match="velocities for every point"):
        executor = object.__new__(ArmMovementExecutor)
        executor._base_result_timeout_sec = 30.0
        executor.result_timeout_sec = 30.0
        executor.moveit_result_timeout_margin_sec = 15.0
        executor._build_moveit_joint_goal(planned)


def test_zero_time_start_gets_lead_time_without_changing_segment_duration(monkeypatch):
    planned = trajectory()
    planned.points[0].time_from_start.nanosec = 0
    captured = capture_joint_move(monkeypatch)

    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0

    executor._build_moveit_joint_goal(planned)

    assert captured["times"] == pytest.approx([0.25, 1.5])
    assert captured["times"][1] - captured["times"][0] == pytest.approx(1.25)
    assert planned.points[0].time_from_start.nanosec == 0


@pytest.mark.parametrize("seconds", [-1, float("nan"), float("inf")])
def test_joint_goal_rejects_invalid_times(monkeypatch, seconds):
    planned = trajectory()
    planned.points[0].time_from_start.sec = seconds
    capture_joint_move(monkeypatch)
    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0
    with pytest.raises(ValueError, match="finite and nonnegative"):
        executor._build_moveit_joint_goal(planned)


def test_moveit_plans_at_full_model_limits_before_execution_retiming():
    assert DEFAULT_VELOCITY_SCALING == pytest.approx(1.0)
    assert DEFAULT_ACCELERATION_SCALING == pytest.approx(1.0)


def test_requested_duration_stretches_moveit_times_and_velocities(monkeypatch):
    planned = trajectory()
    planned.points[0].time_from_start.sec = 0
    planned.points[0].time_from_start.nanosec = 0
    planned.points[1].time_from_start.sec = 1
    planned.points[1].time_from_start.nanosec = 0
    captured = capture_joint_move(monkeypatch)
    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0

    executor._build_moveit_joint_goal(
        planned,
        minimum_duration_sec=2.0,
    )

    assert captured["times"] == pytest.approx([0.25, 2.25])
    assert captured["joint_velocities"] == [
        pytest.approx([0.05, 0.10, 0.15, 0.20, 0.25, 0.30]),
        pytest.approx([0.55, 0.60, 0.65, 0.70, 0.75, 0.80]),
    ]


def test_requested_duration_never_shortens_moveit_plan(monkeypatch):
    captured = capture_joint_move(monkeypatch)
    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0

    executor._build_moveit_joint_goal(
        trajectory(),
        minimum_duration_sec=0.5,
    )

    assert captured["times"] == pytest.approx([0.5, 1.25])


def test_long_moveit_trajectory_extends_result_timeout(monkeypatch):
    planned = trajectory()
    planned.points[0].time_from_start.sec = 0
    planned.points[0].time_from_start.nanosec = 0
    planned.points[1].time_from_start.sec = 1
    planned.points[1].time_from_start.nanosec = 0
    capture_joint_move(monkeypatch)
    executor = object.__new__(ArmMovementExecutor)
    executor._base_result_timeout_sec = 30.0
    executor.result_timeout_sec = 30.0
    executor.moveit_result_timeout_margin_sec = 15.0

    executor._build_moveit_joint_goal(
        planned,
        minimum_duration_sec=45.0,
    )

    assert executor.result_timeout_sec == pytest.approx(60.25)
