"""Describe Spot mobility feedback without changing execution outcomes."""


def base_failure_detail(result, fallback_detail):
    detail = fallback_detail(result)
    command = getattr(getattr(result, "result", None), "command", None)
    if command is None or command.command_choice != command.COMMAND_SYNCHRONIZED_FEEDBACK_SET:
        return detail
    synchronized = command.synchronized_feedback
    if not synchronized.has_field & synchronized.MOBILITY_COMMAND_FEEDBACK_FIELD_SET:
        return detail

    mobility = synchronized.mobility_command_feedback
    diagnostics = [f"mobility={_enum_name(mobility.status, 'STATUS_')}"]
    feedback = mobility.feedback
    if feedback.feedback_choice == feedback.FEEDBACK_STAND_FEEDBACK_SET:
        stand = feedback.stand_feedback
        diagnostics.extend((
            f"stand={_enum_name(stand.status, 'STATUS_')}",
            f"standing_state={_enum_name(stand.standing_state, 'STANDING_')}",
        ))
    elif feedback.feedback_choice == feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET:
        trajectory = feedback.se2_trajectory_feedback
        diagnostics.extend((
            f"trajectory={_enum_name(trajectory.status, 'STATUS_')}",
            f"body_movement={_enum_name(trajectory.body_movement_status, 'BODY_STATUS_')}",
            f"final_goal={_enum_name(trajectory.final_goal_status, 'FINAL_GOAL_STATUS_')}",
        ))
    elif feedback.feedback_choice == feedback.FEEDBACK_SIT_FEEDBACK_SET:
        diagnostics.append(f"sit={_enum_name(feedback.sit_feedback.status, 'STATUS_')}")
    return f"{detail} (Spot {', '.join(diagnostics)})"


def _enum_name(status, prefix):
    for name in dir(status):
        if name.startswith(prefix) and getattr(status, name) == status.value:
            return name
    return f"unknown value {status.value}"
