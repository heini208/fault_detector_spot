"""Translate public operational intents into semantic commands."""

import math

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
)
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
    SemanticTag,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    stamped_pose_from_message,
)


_INTENT_COMMAND_IDS = {
    OperationalIntent.INTENT_STAND_UP: CommandID.STAND_UP,
    OperationalIntent.INTENT_READY_ARM: CommandID.READY_ARM,
    OperationalIntent.INTENT_STOW_ARM: CommandID.STOW_ARM,
    OperationalIntent.INTENT_TOGGLE_GRIPPER: CommandID.TOGGLE_GRIPPER,
    OperationalIntent.INTENT_CLOSE_GRIPPER: CommandID.CLOSE_GRIPPER,
    OperationalIntent.INTENT_RETURN_TO_ESTOP_STATE: (
        CommandID.ESTOP_STATE
    ),
    OperationalIntent.INTENT_STOP_BASE: CommandID.STOP_BASE,
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG: (
        CommandID.MOVE_ARM_TO_TAG
    ),
    OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT: (
        CommandID.MOVE_ARM_TO_TAG_AND_WAIT
    ),
    OperationalIntent.INTENT_SCAN_ALL_IN_RANGE: (
        CommandID.SCAN_ALL_IN_RANGE
    ),
    OperationalIntent.INTENT_MOVE_ARM_RELATIVE: (
        CommandID.MOVE_ARM_RELATIVE
    ),
    OperationalIntent.INTENT_MOVE_BASE_TO_TAG: (
        CommandID.MOVE_BASE_TO_TAG
    ),
    OperationalIntent.INTENT_MOVE_BASE_RELATIVE: (
        CommandID.MOVE_BASE_RELATIVE
    ),
    OperationalIntent.INTENT_MOVE_TO_WAYPOINT: (
        CommandID.MOVE_TO_WAYPOINT
    ),
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
    if (
        normalized < 0.0
        or (normalized == 0.0 and not allow_zero)
    ):
        qualifier = (
            "non-negative" if allow_zero else "positive"
        )
        raise ValueError(
            f"Duration must be {qualifier}"
        )
    return normalized


def _validate_tag(intent: OperationalIntent) -> None:
    _required_text(
        intent.tag.pose.header.frame_id,
        "Tag pose frame",
    )


def _validate_offset(intent: OperationalIntent) -> None:
    _required_text(
        intent.offset.header.frame_id,
        "Offset frame",
    )


def _validate_arm_orientation(
    intent: OperationalIntent,
) -> str:
    value = _required_text(
        intent.orientation_mode,
        "Orientation mode",
    )
    supported = {mode.value for mode in OrientationModes}
    if value not in supported:
        raise ValueError(
            f"Unsupported orientation mode: {value}"
        )
    return value


def operational_intent_to_command(
    intent: OperationalIntent,
) -> SemanticCommand:
    """Validate one public intent and build application command data."""
    if not isinstance(intent, OperationalIntent):
        raise TypeError(
            "Expected an OperationalIntent message"
        )
    try:
        command_id = _INTENT_COMMAND_IDS[int(intent.intent)]
    except (KeyError, TypeError, ValueError) as exception:
        raise ValueError(
            "Unsupported operational intent: "
            f"{intent.intent!r}"
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
    if (
        intent.intent
        == OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT
    ):
        _duration(intent.duration_sec, allow_zero=True)
    if intent.intent == OperationalIntent.INTENT_WAIT:
        _duration(intent.duration_sec)
    if (
        intent.intent
        == OperationalIntent.INTENT_MOVE_TO_WAYPOINT
    ):
        _required_text(intent.map_name, "Map name")
        _required_text(
            intent.waypoint_name,
            "Waypoint name",
        )
    if (
        intent.intent
        == OperationalIntent.INTENT_EXECUTE_PROBE_POINT
    ):
        _required_text(intent.object_id, "Object ID")
        _required_text(intent.routine_id, "Routine ID")
        _required_text(
            intent.probe_point_id,
            "Probe point ID",
        )

    tag = None
    if intent.intent in _TAG_INTENTS:
        tag = SemanticTag(
            id=int(intent.tag.id),
            pose=stamped_pose_from_message(
                intent.tag.pose
            ),
        )

    return SemanticCommand(
        command_id=command_id,
        tag=tag,
        offset=stamped_pose_from_message(intent.offset),
        orientation_mode=intent.orientation_mode,
        wait_time=float(intent.duration_sec),
        map_name=intent.map_name,
        waypoint_name=intent.waypoint_name,
        inspection=InspectionSelection(
            object_id=intent.object_id,
            routine_id=intent.routine_id,
            probe_point_id=intent.probe_point_id,
        ),
    )
