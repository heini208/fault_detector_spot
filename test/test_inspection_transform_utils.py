"""Tests for inspection transform utilities."""

import math

import numpy as np
import pytest
from geometry_msgs.msg import Pose

from fault_detector_spot.inspection.data.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.utility.transform_utils import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
    pose_to_matrix,
    pose_to_pose_data,
    relative_pose,
)


def create_pose(
    x=0.0,
    y=0.0,
    z=0.0,
    yaw=0.0,
):
    """Create a pose with translation and yaw rotation."""
    pose = Pose()

    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def assert_pose_close(
    actual,
    expected,
    tolerance=1e-8,
):
    """Compare poses using transform matrices."""
    assert np.allclose(
        pose_to_matrix(actual),
        pose_to_matrix(expected),
        atol=tolerance,
    )


def test_pose_data_round_trip():
    """PoseData converts to and from a ROS pose."""
    original = PoseData(
        position=Vector3Data(
            x=1.0,
            y=2.0,
            z=3.0,
        ),
        orientation=QuaternionData(
            x=0.0,
            y=0.0,
            z=0.0,
            w=2.0,
        ),
    )

    pose = pose_data_to_pose(original)
    restored = pose_to_pose_data(pose)

    assert restored.position == original.position
    assert restored.orientation == QuaternionData(
        x=0.0,
        y=0.0,
        z=0.0,
        w=1.0,
    )


def test_translation_composition():
    """Translations compose in frame-chain order."""
    map_to_marker = create_pose(
        x=1.0,
        y=2.0,
        z=0.0,
    )
    marker_to_object = create_pose(
        x=0.5,
        y=0.0,
        z=0.2,
    )

    result = compose_poses(
        map_to_marker,
        marker_to_object,
    )

    expected = create_pose(
        x=1.5,
        y=2.0,
        z=0.2,
    )

    assert_pose_close(result, expected)


def test_rotated_translation_composition():
    """Child translation is rotated by the parent frame."""
    map_to_marker = create_pose(
        x=1.0,
        y=2.0,
        yaw=math.pi / 2.0,
    )
    marker_to_object = create_pose(
        x=1.0,
        y=0.0,
    )

    result = compose_poses(
        map_to_marker,
        marker_to_object,
    )

    expected = create_pose(
        x=1.0,
        y=3.0,
        yaw=math.pi / 2.0,
    )

    assert_pose_close(result, expected)


def test_pose_times_inverse_is_identity():
    """A pose composed with its inverse is identity."""
    pose = create_pose(
        x=1.2,
        y=-0.4,
        z=0.8,
        yaw=0.7,
    )

    result = compose_poses(
        pose,
        inverse_pose(pose),
    )

    assert_pose_close(result, create_pose())


def test_relative_pose_recovers_child_transform():
    """Relative pose recovers the stored child transform."""
    map_to_object = create_pose(
        x=3.0,
        y=4.0,
        yaw=0.4,
    )
    object_to_base = create_pose(
        x=-1.0,
        y=0.2,
        yaw=-0.1,
    )

    map_to_base = compose_poses(
        map_to_object,
        object_to_base,
    )

    recovered = relative_pose(
        map_to_object,
        map_to_base,
    )

    assert_pose_close(
        recovered,
        object_to_base,
    )


def test_zero_quaternion_is_rejected():
    """A zero-length quaternion is invalid."""
    pose = Pose()
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

    with pytest.raises(
        ValueError,
        match="Quaternion norm is zero",
    ):
        pose_to_matrix(pose)