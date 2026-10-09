"""Saved probe targets retain object coordinates until execution."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeedPolicy
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)
from fault_detector_spot.manipulation.probe_motion_planner import ProbeMotionPlanner
from fault_detector_spot.shared.geometry.movement_frames import OrientationModes
from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable


class BodyTF:
    """Provide robot TF, failing if a second tag observation is requested."""

    def __init__(self):
        self.lookups = []
        self.checks = []
        self._tf_buffer = SimpleNamespace(can_transform=self.can_transform)

    def can_transform(self, target, source, _time):
        self.checks.append((target, source))
        return {target, source} <= {"body", "flat_body"}

    def lookup_a_tform_b(self, target, source, **_kwargs):
        self.lookups.append((target, source))
        assert {target, source} <= {"body", "flat_body"}
        result = TransformStamped()
        result.header.frame_id = target
        result.child_frame_id = source
        result.transform.rotation.w = 1.0
        return result


def set_orientation(pose, rotation):
    quaternion = pose.orientation
    quaternion.x, quaternion.y, quaternion.z, quaternion.w = map(
        float, rotation.as_quat(),
    )


def observed_tag(rotation=None):
    pose = PoseStamped()
    pose.header.frame_id = "body"
    pose.header.stamp.sec = 23
    pose.pose.position.x = 10.0
    pose.pose.position.y = 20.0
    pose.pose.position.z = 30.0
    set_orientation(pose.pose, Rotation.identity() if rotation is None else rotation)
    return SimpleNamespace(id=7, pose=pose)


def saved_command(mode=OrientationModes.CUSTOM_ORIENTATION):
    offset = PoseStamped()
    offset.header.frame_id = "filtered_fiducial_7"
    offset.pose.position.x = 1.0
    offset.pose.position.y = 2.0
    offset.pose.position.z = 3.0
    set_orientation(offset.pose, Rotation.from_euler("xyz", [0.2, -0.1, 0.3]))
    return ManipulatorToTagCommand(
        CommandID.MOVE_ARM_TO_TAG, Time(), PoseStamped(), 7,
        offset=offset, orientation_mode=mode, motion_sensor_id="probe",
    )


def quaternion(pose):
    value = pose.orientation
    return [value.x, value.y, value.z, value.w]


def position(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


@pytest.mark.parametrize(
    "axis, expected_position",
    [("x", [11, 17, 32]), ("y", [13, 22, 29]), ("z", [8, 21, 33])],
)
def test_saved_target_uses_full_live_tag_rotation(axis, expected_position):
    listener = BodyTF()
    planner = ProbeMotionPlanner(listener, ArmMotionSpeedPolicy())
    command = saved_command()
    rotation = Rotation.from_euler(axis, 90, degrees=True)
    source = SimpleNamespace(usable_tag=lambda _id: observed_tag(rotation))

    result = planner.resolve_tag(command, source)

    assert position(result.target.pose) == pytest.approx(expected_position)
    expected_orientation = rotation * Rotation.from_quat(quaternion(command.offset.pose))
    assert quaternion(result.target.pose) == pytest.approx(expected_orientation.as_quat())
    assert result.sensor_id == "probe"
    assert result.target.header.frame_id == "body"
    assert listener.lookups == []
    assert all("fiducial" not in frame for pair in listener.checks for frame in pair)


def test_repeated_resolution_uses_new_tag_without_mutating_saved_target():
    planner = ProbeMotionPlanner(BodyTF(), ArmMotionSpeedPolicy())
    command = saved_command()
    original_offset = deepcopy(command.offset)
    original_tag = deepcopy(command.tag_pose)
    observation = observed_tag()
    source = SimpleNamespace(usable_tag=lambda _id: observation)

    first = planner.resolve_tag(command, source)
    observation = observed_tag(Rotation.from_euler("z", 90, degrees=True))
    observation.pose.pose.position.x = 40.0
    second = planner.resolve_tag(command, source)

    assert position(first.target.pose) == pytest.approx([11, 22, 33])
    assert position(second.target.pose) == pytest.approx([38, 21, 33])
    assert command.offset == original_offset
    assert command.tag_pose == original_tag


def test_saved_target_requires_usable_tag_before_resolving_geometry():
    listener = BodyTF()
    planner = ProbeMotionPlanner(listener, ArmMotionSpeedPolicy())
    source = SimpleNamespace(usable_tag=lambda _id: None)

    with pytest.raises(RuntimeError, match="Tag 7 is not currently usable"):
        planner.resolve_tag(saved_command(), source)

    assert listener.checks == []
    assert listener.lookups == []


def test_tag_relative_orientation_keeps_existing_facing_convention():
    planner = ProbeMotionPlanner(BodyTF(), ArmMotionSpeedPolicy())
    command = saved_command(OrientationModes.TAG_ORIENTATION)
    rotation = Rotation.from_euler("xyz", [0.4, -0.2, 0.6])
    source = SimpleNamespace(usable_tag=lambda _id: observed_tag(rotation))

    result = planner.resolve_tag(command, source)

    expected = (rotation * Rotation.from_euler("y", 90, degrees=True)
                * Rotation.from_quat(quaternion(command.offset.pose)))
    assert quaternion(result.target.pose) == pytest.approx(expected.as_quat())


def test_body_relative_offset_keeps_existing_geometry():
    planner = ProbeMotionPlanner(BodyTF(), ArmMotionSpeedPolicy())
    command = saved_command()
    command.offset.header.frame_id = "body"
    source = SimpleNamespace(
        usable_tag=lambda _id: observed_tag(Rotation.from_euler("z", 90, degrees=True)),
    )

    result = planner.resolve_tag(command, source)

    assert position(result.target.pose) == pytest.approx([11, 22, 33])
    assert quaternion(result.target.pose) == pytest.approx(quaternion(command.offset.pose))


@pytest.mark.parametrize("frame", ["fiducial_7", "tag36h11:7", "filtered_fiducial_8", "Tag_7"])
def test_other_tag_frames_keep_their_existing_tf_requirements(frame):
    planner = ProbeMotionPlanner(BodyTF(), ArmMotionSpeedPolicy())
    command = saved_command()
    command.offset.header.frame_id = frame
    source = SimpleNamespace(usable_tag=lambda _id: observed_tag())

    with pytest.raises(MovementGeometryUnavailable):
        planner.resolve_tag(command, source)
