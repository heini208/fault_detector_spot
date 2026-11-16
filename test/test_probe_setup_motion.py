"""Tests for single-step probe setup command translation."""

import math

import pytest
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
)
from fault_detector_spot.shared.geometry.movement_frames import (
    OrientationModes,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.shared.geometry.models import (
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


def test_surface_orientation_reuses_existing_semantic_command():
    command = ProbeSetupMotionCommandFactory().orient_to_surface(
        "hall_probe"
    )

    assert isinstance(command, SemanticCommand)
    assert command.command_id is CommandID.ORIENT_TO_SURFACE
    assert command.motion_sensor_id == "hall_probe"


def test_tag_orientation_reuses_existing_semantic_command():
    command = ProbeSetupMotionCommandFactory().orient_to_tag(
        tag(),
        "hall_probe",
    )

    assert isinstance(command, SemanticCommand)
    assert command.command_id is CommandID.ORIENT_TO_TAG
    assert command.tag.id == 7
    assert command.motion_sensor_id == "hall_probe"


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
        translation=Vector3Data(x=0.201, y=0.0, z=0.0),
    )

    with pytest.raises(ValueError, match="exceeds 0.2 m"):
        request.validate()


@pytest.mark.parametrize("kind", [
    ProbeMotionKind.ADJUST_SAFE_APPROACH,
    ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH,
    ProbeMotionKind.ADJUST_PATHING_POSE,
])
def test_fine_adjustment_accepts_two_tenths_of_a_metre(kind):
    ProbeMotionRequest(
        kind=kind, translation=Vector3Data(x=0.2, y=-0.2, z=0.2),
    ).validate()


@pytest.mark.parametrize("direction, expected", [
    ((1, 0, 0), (0, 0, -1)),  # Forward approaches the tag.
    ((-1, 0, 0), (0, 0, 1)),
    ((0, 1, 0), (0, 1, 0)),
    ((0, -1, 0), (0, -1, 0)),
    ((0, 0, 1), (1, 0, 0)),  # Up follows the tag's upright axis.
    ((0, 0, -1), (-1, 0, 0)),
])
def test_tag_fine_adjustment_uses_tag_facing_axes(direction, expected):
    command = ProbeSetupMotionCommandFactory().relative(
        "filtered_fiducial_7", Vector3Data(*(v * .01 for v in direction)),
        0.0, 0.0, "hall_probe", adjustment_frame=ProbeMotionFrame.TAG,
    )
    position = command.offset.position
    assert (position.x, position.y, position.z) == pytest.approx(
        tuple(v * .01 for v in expected), abs=1e-12,
    )
    assert command.offset.frame_id == "filtered_fiducial_7"
    assert command.offset.orientation.w == pytest.approx(1.0)


@pytest.mark.parametrize("pitch, yaw, expected_axis", [
    (-.02, 0, (0, -1, 0)),  # Pitch up rotates forward (-Z) toward up (+X).
    (.02, 0, (0, 1, 0)),
    (0, .02, (1, 0, 0)),  # Yaw left rotates forward (-Z) toward left (+Y).
    (0, -.02, (-1, 0, 0)),
])
def test_tag_rotation_adjustments_rotate_about_tag_facing_axes(pitch, yaw, expected_axis):
    command = ProbeSetupMotionCommandFactory().relative(
        "filtered_fiducial_7", Vector3Data.zero(), pitch, yaw, "hall_probe",
        adjustment_frame=ProbeMotionFrame.TAG,
    )
    rotation = command.offset.orientation
    assert (rotation.x, rotation.y, rotation.z) == pytest.approx(
        tuple(v * math.sin(.01) for v in expected_axis), abs=1e-12,
    )
    assert rotation.w == pytest.approx(math.cos(.01))


@pytest.mark.parametrize("frame", [
    ProbeMotionFrame.SENSOR, ProbeMotionFrame.HAND, ProbeMotionFrame.BODY, ProbeMotionFrame.MAP,
])
def test_other_fine_adjustment_frames_preserve_forward_axis(frame):
    command = ProbeSetupMotionCommandFactory().relative(
        ProbeSetupMotionCommandFactory.frame_id(frame, "hall_probe", 7),
        Vector3Data(.01, 0, 0), 0.0, 0.0, "hall_probe", adjustment_frame=frame,
    )
    position = command.offset.position
    assert (position.x, position.y, position.z) == (.01, 0, 0)
