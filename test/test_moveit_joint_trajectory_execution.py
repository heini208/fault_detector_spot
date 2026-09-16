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
    executor._moveit_cartesian_plan = object()
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

    ArmMovementExecutor._build_moveit_joint_goal(
        trajectory(with_velocities=False)
    )

    assert captured["joint_velocities"] is None


def test_moveit_joint_goal_rejects_partial_velocity_data(monkeypatch):
    planned = trajectory()
    planned.points[1].velocities = []
    capture_joint_move(monkeypatch)

    with pytest.raises(ValueError, match="velocities for every point"):
        ArmMovementExecutor._build_moveit_joint_goal(planned)
