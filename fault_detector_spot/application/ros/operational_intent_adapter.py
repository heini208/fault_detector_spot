"""Translate public operational intents into internal commands."""

from copy import deepcopy
import math

from fault_detector_msgs.msg import ComplexCommand, OperationalIntent

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
)


_INTENT_COMMAND_IDS = {
    OperationalIntent.INTENT_STAND_UP: CommandID.STAND_UP,
    OperationalIntent.INTENT_READY_ARM: CommandID.READY_ARM,
    OperationalIntent.INTENT_STOW_ARM: CommandID.STOW_ARM,
    OperationalIntent.INTENT_TOGGLE_GRIPPER: CommandID.TOGGLE_GRIPPER,
    OperationalIntent.INTENT_CLOSE_GRIPPER: CommandID.CLOSE_GRIPPER,
    OperationalIntent.INTENT_RETURN_TO_ESTOP_STATE: CommandID.ESTOP_STATE,
    OperationalIntent.INTENT_STOP_BASE: CommandID.STOP_BASE,
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG: CommandID.MOVE_ARM_TO_TAG,
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT: (
        CommandID.MOVE_ARM_TO_TAG_AND_WAIT
    ),
    OperationalIntent.INTENT_SCAN_ALL_IN_RANGE: CommandID.SCAN_ALL_IN_RANGE,
    OperationalIntent.INTENT_MOVE_ARM_RELATIVE: CommandID.MOVE_ARM_RELATIVE,
    OperationalIntent.INTENT_MOVE_BASE_TO_TAG: CommandID.MOVE_BASE_TO_TAG,
    OperationalIntent.INTENT_MOVE_BASE_RELATIVE: CommandID.MOVE_BASE_RELATIVE,
    OperationalIntent.INTENT_MOVE_TO_WAYPOINT: CommandID.MOVE_TO_WAYPOINT,
    OperationalIntent.INTENT_WAIT: CommandID.WAIT_TIME,
    OperationalIntent.INTENT_EXECUTE_PROBE_POINT: (
        CommandID.EXECUTE_PROBE_POINT
    ),
}

_TAG_INTENTS = frozenset({
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG,
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT,
    OperationalIntent.INTENT_MOVE_BASE_TO_TAG,
})

_OFFSET_INTENTS = frozenset({
    OperationalIntent.INTENT_MOVE_ARM_RELATIVE,
    OperationalIntent.INTENT_MOVE_BASE_RELATIVE,
})


def _required_text(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{label} must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _duration(value: float, allow_zero: bool = False) -> float:
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError("Duration must be finite")
    if normalized < 0.0 or (normalized == 0.0 and not allow_zero):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"Duration must be {qualifier}")
    return normalized


def _validate_tag(intent: OperationalIntent) -> None:
    _required_text(intent.tag.pose.header.frame_id, "Tag pose frame")


def _validate_offset(intent: OperationalIntent) -> None:
    _required_text(intent.offset.header.frame_id, "Offset frame")


def _validate_arm_orientation(intent: OperationalIntent) -> str:
    value = _required_text(intent.orientation_mode, "Orientation mode")
    supported = {mode.value for mode in OrientationModes}
    if value not in supported:
        raise ValueError(f"Unsupported orientation mode: {value}")
    return value


def operational_intent_to_command(
    intent: OperationalIntent,
) -> ComplexCommand:
    """Validate one public intent and build its internal command payload."""
    if not isinstance(intent, OperationalIntent):
        raise TypeError("Expected an OperationalIntent message")
    try:
        command_id = _INTENT_COMMAND_IDS[int(intent.intent)]
    except (KeyError, TypeError, ValueError) as exception:
        raise ValueError(
            f"Unsupported operational intent: {intent.intent!r}"
        ) from exception

    if intent.intent in _TAG_INTENTS:
        _validate_tag(intent)
    if intent.intent in _OFFSET_INTENTS:
        _validate_offset(intent)
    if intent.intent in (
        OperationalIntent.INTENT_MOVE_ARM_TO_TAG,
        OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT,
    ):
        _validate_arm_orientation(intent)
    if intent.intent == OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT:
        _duration(intent.duration_sec, allow_zero=True)
    if intent.intent == OperationalIntent.INTENT_WAIT:
        _duration(intent.duration_sec)
    if intent.intent == OperationalIntent.INTENT_MOVE_TO_WAYPOINT:
        _required_text(intent.map_name, "Map name")
        _required_text(intent.waypoint_name, "Waypoint name")
    if intent.intent == OperationalIntent.INTENT_EXECUTE_PROBE_POINT:
        _required_text(intent.object_id, "Object ID")
        _required_text(intent.routine_id, "Routine ID")
        _required_text(intent.probe_point_id, "Probe point ID")

    command = ComplexCommand()
    command.command.command_id = command_id.value
    command.tag = deepcopy(intent.tag)
    command.offset = deepcopy(intent.offset)
    command.orientation_mode = intent.orientation_mode.strip()
    command.wait_time = float(intent.duration_sec)
    command.map_name = intent.map_name.strip()
    command.waypoint_name = intent.waypoint_name.strip()
    command.inspection.object_id = intent.object_id.strip()
    command.inspection.routine_id = intent.routine_id.strip()
    command.inspection.probe_point_id = intent.probe_point_id.strip()
    return command
