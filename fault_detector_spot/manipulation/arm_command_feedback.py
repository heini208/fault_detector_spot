"""Decode Spot command failures without changing execution state."""

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome

def arm_failure_result(result, fallback_detail):
    command = getattr(getattr(result, "result", None), "command", None)
    if (
        getattr(command, "command_choice", None) == 0
        and not getattr(result, "message", "")
        and not getattr(result, "detail", "")
    ):
        return (
            ArmMovementOutcome.MOTION_FAILED,
            "Spot RobotCommand failed without feedback or an error "
            "message; check spot_driver logs for the underlying failure",
        )
    feedback = _cartesian_feedback(result)
    status = getattr(feedback, "status", None)
    value = getattr(status, "value", None)

    if _status_matches(
        status,
        value,
        "STATUS_TRAJECTORY_STALLED",
    ):
        return (
            ArmMovementOutcome.TRAJECTORY_STALLED,
            _cartesian_failure_detail(
                "Cartesian arm trajectory stalled",
                result,
            ),
        )

    if _status_matches(
        status,
        value,
        "STATUS_TRAJECTORY_CANCELLED",
    ):
        return (
            ArmMovementOutcome.TRAJECTORY_CANCELLED,
            _cartesian_failure_detail(
                "Cartesian arm trajectory cancelled",
                result,
            ),
        )

    return (
        ArmMovementOutcome.MOTION_FAILED,
        fallback_detail(result),
    )


def _cartesian_failure_detail(prefix: str, result) -> str:
    detail = str(
        getattr(result, "message", "")
        or getattr(result, "detail", "")
    ).strip()
    if not detail:
        return prefix
    return f"{prefix}; {detail}"


def _status_matches(status, value, constant_name: str) -> bool:
    if status is None or value is None:
        return False
    expected = getattr(status, constant_name, None)
    return expected is not None and value == expected


def _cartesian_feedback(result):
    command_feedback = getattr(result, "result", None)
    command = getattr(command_feedback, "command", None)
    synchronized = getattr(command, "synchronized_feedback", None)
    arm = getattr(synchronized, "arm_command_feedback", None)
    feedback = getattr(arm, "feedback", None)
    if feedback is None:
        return None

    cartesian_choice = getattr(
        feedback,
        "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET",
        None,
    )
    feedback_choice = getattr(feedback, "feedback_choice", None)
    if (
        cartesian_choice is not None
        and feedback_choice != cartesian_choice
    ):
        return None

    return getattr(feedback, "arm_cartesian_feedback", None)



