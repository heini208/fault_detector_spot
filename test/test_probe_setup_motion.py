"""Tests for single-step probe setup command translation."""

import math

import pytest
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionFrame,
    ProbeMotionKind,
    ProbeMotionRequest,
    ProbeSetupMotionCommandFactory,
)


def pose(x=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )


def tag(x=1.0):
    message = TagElement()
    message.id = 7
    message.pose.header.frame_id = "body"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def test_absolute_motion_is_one_probe_target_semantic_arm_primitive():
    command = ProbeSetupMotionCommandFactory().absolute(
        pose(0.5),
        tag(),
        "hall_probe",
    )

    assert isinstance(command, SemanticCommand)
    assert command.command_id is CommandID.MOVE_ARM_TO_TAG
    assert command.tag.id == 7
    assert command.offset.frame_id == "body"
    assert command.offset.position.x == pytest.approx(0.5)
    assert command.offset.orientation.w == pytest.approx(1.0)
    assert command.motion_sensor_id == "hall_probe"
    assert (
        command.orientation_mode
        == OrientationModes.CUSTOM_ORIENTATION.value
    )


def test_tag_aligned_absolute_motion_uses_relative_to_tag_with_zero_rotation():
    command = ProbeSetupMotionCommandFactory().absolute(
        PoseData(
            position=Vector3Data(x=0.5, y=0.2, z=-0.1),
            orientation=QuaternionData(
                x=0.1,
                y=0.2,
                z=0.3,
                w=0.9,
            ),
        ),
        tag(),
        "hall_probe",
        orientation_mode=OrientationModes.TAG_ORIENTATION.value,
    )

    assert command.command_id is CommandID.MOVE_ARM_TO_TAG
    assert command.orientation_mode == OrientationModes.TAG_ORIENTATION.value
    assert command.offset.position.x == pytest.approx(0.5)
    assert command.offset.position.y == pytest.approx(0.2)
    assert command.offset.position.z == pytest.approx(-0.1)
    assert command.offset.orientation.x == pytest.approx(0.0)
    assert command.offset.orientation.y == pytest.approx(0.0)
    assert command.offset.orientation.z == pytest.approx(0.0)
    assert command.offset.orientation.w == pytest.approx(1.0)


def test_relative_motion_is_one_semantic_relative_arm_primitive():
    command = ProbeSetupMotionCommandFactory().relative(
        "probe_hall_probe",
        Vector3Data(x=0.01, y=-0.02, z=0.0),
        pitch_rad=math.radians(2.0),
        yaw_rad=math.radians(-3.0),
        motion_sensor_id="hall_probe",
    )

    assert isinstance(command, SemanticCommand)
    assert command.command_id is CommandID.MOVE_ARM_RELATIVE
    assert command.offset.frame_id == "probe_hall_probe"
    assert command.offset.position.x == pytest.approx(0.01)
    assert command.offset.position.y == pytest.approx(-0.02)
    assert command.motion_sensor_id == "hall_probe"


def test_sensor_relative_frame_uses_real_hand_when_no_sensor_is_attached():
    frame = ProbeSetupMotionCommandFactory.frame_id(
        ProbeMotionFrame.SENSOR,
        "hand",
        7,
    )

    assert frame == "hand"


def test_motion_request_rejects_oversized_manual_adjustment():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.ADJUST_SAFE_APPROACH,
        frame=ProbeMotionFrame.SENSOR,
        translation=Vector3Data(x=0.051, y=0.0, z=0.0),
    )

    with pytest.raises(ValueError, match="exceeds 0.05 m"):
        request.validate()
