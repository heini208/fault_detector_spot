"""Tests for single-step probe setup command translation."""

import math

import pytest
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
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


def test_absolute_motion_is_one_existing_arm_primitive():
    command = ProbeSetupMotionCommandFactory().absolute(
        pose(0.5),
        PoseData.identity(),
        tag(),
    )

    assert command.command.command_id == CommandID.MOVE_ARM_TO_TAG.value
    assert command.tag.id == 7
    assert command.offset.header.frame_id == "body"
    assert command.offset.pose.position.x == pytest.approx(0.5)
    assert command.offset.pose.orientation.w == pytest.approx(1.0)
    assert (
        command.orientation_mode
        == OrientationModes.CUSTOM_ORIENTATION.value
    )


def test_relative_motion_is_one_existing_relative_arm_primitive():
    command = ProbeSetupMotionCommandFactory().relative(
        "probe_hall_probe",
        Vector3Data(x=0.01, y=-0.02, z=0.0),
        pitch_rad=math.radians(2.0),
        yaw_rad=math.radians(-3.0),
    )

    assert command.command.command_id == CommandID.MOVE_ARM_RELATIVE.value
    assert command.offset.header.frame_id == "probe_hall_probe"
    assert command.offset.pose.position.x == pytest.approx(0.01)
    assert command.offset.pose.position.y == pytest.approx(-0.02)


def test_motion_request_rejects_oversized_manual_adjustment():
    request = ProbeMotionRequest(
        kind=ProbeMotionKind.ADJUST_SAFE_APPROACH,
        frame=ProbeMotionFrame.SENSOR,
        translation=Vector3Data(x=0.051, y=0.0, z=0.0),
    )

    with pytest.raises(ValueError, match="exceeds 0.05 m"):
        request.validate()
