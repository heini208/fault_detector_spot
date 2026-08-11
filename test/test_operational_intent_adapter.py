import pytest

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)


def test_translates_simple_operational_intent():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_STAND_UP

    command = operational_intent_to_command(intent)

    assert command.command.command_id == CommandID.STAND_UP.value


def test_preserves_tag_motion_payload():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_ARM_TO_TAG_AND_WAIT
    intent.tag.id = 12
    intent.tag.pose.header.frame_id = "odom"
    intent.tag.pose.pose.position.x = 1.25
    intent.offset.header.frame_id = "body"
    intent.offset.pose.position.z = 0.15
    intent.orientation_mode = "relative_to_tag"
    intent.duration_sec = 3.5

    command = operational_intent_to_command(intent)

    assert command.command.command_id == (
        CommandID.MOVE_ARM_TO_TAG_AND_WAIT.value
    )
    assert command.tag.id == 12
    assert command.tag.pose.header.frame_id == "odom"
    assert command.tag.pose.pose.position.x == 1.25
    assert command.offset.header.frame_id == "body"
    assert command.offset.pose.position.z == 0.15
    assert command.orientation_mode == "relative_to_tag"
    assert command.wait_time == 3.5


def test_requires_frame_for_relative_motion():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_BASE_RELATIVE

    with pytest.raises(ValueError, match="Offset frame"):
        operational_intent_to_command(intent)


def test_requires_positive_explicit_wait():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_WAIT
    intent.duration_sec = 0.0

    with pytest.raises(ValueError, match="positive"):
        operational_intent_to_command(intent)


def test_requires_complete_waypoint_identity():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_TO_WAYPOINT
    intent.map_name = "factory"

    with pytest.raises(ValueError, match="Waypoint name"):
        operational_intent_to_command(intent)


def test_translates_probe_execution_selection():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_EXECUTE_PROBE_POINT
    intent.object_id = "motor_a"
    intent.routine_id = "magnetic_scan"
    intent.probe_point_id = "bearing_1"

    command = operational_intent_to_command(intent)

    assert command.command.command_id == CommandID.EXECUTE_PROBE_POINT.value
    assert command.inspection.object.object_id == "motor_a"
    assert command.inspection.routine.routine_id == "magnetic_scan"
    assert command.inspection.probe_point_id == "bearing_1"


def test_rejects_unspecified_intent():
    intent = OperationalIntent()

    with pytest.raises(ValueError, match="Unsupported operational intent"):
        operational_intent_to_command(intent)


def test_public_contract_excludes_setup_intents():
    assert not hasattr(OperationalIntent, "INTENT_CREATE_MAP")
    assert not hasattr(OperationalIntent, "INTENT_START_SLAM")
    assert not hasattr(OperationalIntent, "INTENT_CAPTURE_REFERENCE_VIEW")
