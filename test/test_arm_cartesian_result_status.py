"""Typed Spot Cartesian arm trajectory result tests."""

from types import SimpleNamespace

from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
    _ArmOperation,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
)


class Status:

    STATUS_TRAJECTORY_COMPLETE = 2
    STATUS_TRAJECTORY_CANCELLED = 3
    STATUS_TRAJECTORY_STALLED = 4

    def __init__(self, value):
        self.value = value


def action_result(status_value, *, cartesian=True, message=""):
    feedback = SimpleNamespace(
        FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET=3,
        feedback_choice=3 if cartesian else 2,
        arm_cartesian_feedback=SimpleNamespace(
            status=Status(status_value),
        ),
    )
    return SimpleNamespace(
        success=status_value == Status.STATUS_TRAJECTORY_COMPLETE,
        message=message,
        result=SimpleNamespace(
            command=SimpleNamespace(
                synchronized_feedback=SimpleNamespace(
                    arm_command_feedback=SimpleNamespace(
                        feedback=feedback,
                    ),
                ),
            ),
        ),
    )


def executor_shell(operation=_ArmOperation.MOVEMENT):
    executor = object.__new__(ArmMovementExecutor)
    executor._operation = operation
    executor._send_goal_future = object()
    executor._goal_handle = object()
    executor._result_future = object()
    executor._goal_sent_monotonic = 1.0
    executor._result_started_monotonic = 2.0
    return executor


def test_cartesian_stall_maps_to_typed_outcome():
    executor = executor_shell()
    result = action_result(
        Status.STATUS_TRAJECTORY_STALLED,
        message="Failed to complete command",
    )

    outcome, detail = executor._arm_failure_result(result)

    assert outcome is ArmMovementOutcome.TRAJECTORY_STALLED
    assert "trajectory stalled" in detail.lower()
    assert "Failed to complete command" in detail


def test_cartesian_cancel_maps_to_typed_outcome():
    executor = executor_shell()
    result = action_result(
        Status.STATUS_TRAJECTORY_CANCELLED,
        message="Cancelled",
    )

    outcome, detail = executor._arm_failure_result(result)

    assert outcome is ArmMovementOutcome.TRAJECTORY_CANCELLED
    assert "trajectory cancelled" in detail.lower()
    assert "Cancelled" in detail


def test_non_cartesian_failure_keeps_generic_motion_failure():
    executor = executor_shell()
    result = action_result(
        Status.STATUS_TRAJECTORY_STALLED,
        cartesian=False,
        message="Failed to complete command",
    )

    outcome, detail = executor._arm_failure_result(result)

    assert outcome is ArmMovementOutcome.MOTION_FAILED
    assert detail == "Failed to complete command"


def test_guarded_failure_releases_only_goal_lifecycle():
    executor = executor_shell(_ArmOperation.GUARDED_MOVEMENT)
    result = action_result(Status.STATUS_TRAJECTORY_STALLED)

    update = executor._handle_failed_result(result)

    assert update.outcome is ArmMovementOutcome.TRAJECTORY_STALLED
    assert executor._operation == _ArmOperation.GUARDED_MOVEMENT
    assert executor._send_goal_future is None
    assert executor._goal_handle is None
    assert executor._result_future is None
    assert executor._goal_sent_monotonic is None
    assert executor._result_started_monotonic is None
