"""Failed base actions retain driver detail and expose the SDK failure cause."""

from types import SimpleNamespace

import pytest
from bosdyn.api import basic_command_pb2, robot_command_pb2
from bosdyn_spot_api_msgs.conversions import convert
from spot_msgs.action import RobotCommand

from fault_detector_spot.navigation.base_command_feedback import base_failure_detail
from fault_detector_spot.navigation.base_movement_executor import BaseMovementOutcome
from fault_detector_spot.shared.execution.movement_executor import MovementExecutor
from test_base_walking_completion import WalkingRig


def failed_result(status="STATUS_COMMAND_OVERRIDDEN", feedback_kind="stand"):
    native = robot_command_pb2.RobotCommandFeedback()
    mobility = native.synchronized_feedback.mobility_command_feedback
    mobility.status = getattr(basic_command_pb2.RobotCommandFeedbackStatus, status)
    if feedback_kind == "stand":
        stand = mobility.stand_feedback
        stand.status = stand.STATUS_IS_STANDING
        stand.standing_state = stand.STANDING_CONTROLLED
    elif feedback_kind == "trajectory":
        trajectory = mobility.se2_trajectory_feedback
        trajectory.status = trajectory.STATUS_AT_GOAL
        trajectory.body_movement_status = trajectory.BODY_STATUS_SETTLED
        trajectory.final_goal_status = trajectory.FINAL_GOAL_STATUS_BLOCKED
    result = RobotCommand.Result(success=False, message="Failed to complete command")
    convert(native, result.result)
    return result


@pytest.mark.parametrize("status", [
    "STATUS_COMMAND_OVERRIDDEN", "STATUS_COMMAND_TIMED_OUT",
    "STATUS_ROBOT_FROZEN", "STATUS_INCOMPATIBLE_HARDWARE",
])
def test_mobility_failure_retains_driver_message_and_exact_status(status):
    result = failed_result(status)
    detail = base_failure_detail(result, MovementExecutor._command_failure_detail)
    assert detail.startswith("Failed to complete command")
    assert f"mobility={status}" in detail
    assert "stand=STATUS_IS_STANDING" in detail
    assert "standing_state=STANDING_CONTROLLED" in detail


def test_trajectory_failure_includes_physical_settling_and_final_goal_feedback():
    result = failed_result(feedback_kind="trajectory")
    detail = base_failure_detail(result, MovementExecutor._command_failure_detail)
    assert "trajectory=STATUS_AT_GOAL" in detail
    assert "body_movement=BODY_STATUS_SETTLED" in detail
    assert "final_goal=FINAL_GOAL_STATUS_BLOCKED" in detail
    assert "stand=" not in detail


@pytest.mark.parametrize("feedback", ["missing", "empty", "full_body", "arm_only"])
def test_absent_mobility_feedback_keeps_original_failure_detail(feedback):
    result = RobotCommand.Result(success=False, message="Original driver error")
    if feedback == "missing":
        result = SimpleNamespace(message="Original driver error")
    elif feedback == "full_body":
        command = result.result.command
        command.command_choice = command.COMMAND_FULL_BODY_FEEDBACK_SET
    elif feedback == "arm_only":
        command = result.result.command
        command.command_choice = command.COMMAND_SYNCHRONIZED_FEEDBACK_SET
        command.synchronized_feedback.has_field = (
            command.synchronized_feedback.ARM_COMMAND_FEEDBACK_FIELD_SET
        )
    assert base_failure_detail(result, MovementExecutor._command_failure_detail) == (
        "Original driver error"
    )


def test_unrecognized_sdk_status_retains_numeric_value():
    result = failed_result(feedback_kind=None)
    result.result.command.synchronized_feedback.mobility_command_feedback.status.value = 99
    detail = base_failure_detail(result, MovementExecutor._command_failure_detail)
    assert "mobility=unknown value 99" in detail


def test_failed_stand_at_reached_waypoint_stays_failed_without_retry():
    rig = WalkingRig()
    rig.executor.finish_walking()
    rig.accept()
    rig.results[-1].set_result(SimpleNamespace(result=failed_result()))
    update = rig.executor.poll()
    assert update.outcome is BaseMovementOutcome.MOTION_FAILED
    assert "STATUS_COMMAND_OVERRIDDEN" in update.detail
    assert "STATUS_IS_STANDING" in update.detail
    assert not rig.executor.active
    assert len(rig.goals) == 1
