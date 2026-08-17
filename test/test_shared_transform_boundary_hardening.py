"""Boundary validation for shared ROS pose transforms."""

import math

import numpy as np
import pytest
from geometry_msgs.msg import Pose

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    matrix_to_pose,
    normalized_quaternion,
    pose_data_to_pose,
    pose_to_matrix,
    pose_to_pose_data,
)


def valid_pose():
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.w = 1.0
    return pose


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_normalized_quaternion_rejects_non_finite_values(value):
    with pytest.raises(ValueError, match="non-finite"):
        normalized_quaternion(value, 0.0, 0.0, 1.0)


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_pose_to_matrix_rejects_non_finite_translation(axis):
    pose = valid_pose()
    setattr(pose.position, axis, float("nan"))

    with pytest.raises(ValueError, match="position contains a non-finite"):
        pose_to_matrix(pose)


def test_pose_to_pose_data_rejects_non_finite_translation():
    pose = valid_pose()
    pose.position.x = float("inf")

    with pytest.raises(ValueError, match="position contains a non-finite"):
        pose_to_pose_data(pose)


def test_pose_data_to_pose_validates_source_data():
    data = PoseData(
        position=Vector3Data(x=float("nan"), y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )

    with pytest.raises(ValueError, match="Vector contains a non-finite"):
        pose_data_to_pose(data)


def test_matrix_to_pose_rejects_invalid_homogeneous_row():
    matrix = np.eye(4)
    matrix[3, 0] = 0.1

    with pytest.raises(ValueError, match="homogeneous row"):
        matrix_to_pose(matrix)


def test_matrix_to_pose_rejects_scaled_rotation_matrix():
    matrix = np.eye(4)
    matrix[0, 0] = 2.0

    with pytest.raises(ValueError, match="not orthonormal"):
        matrix_to_pose(matrix)


def test_matrix_to_pose_rejects_reflection():
    matrix = np.eye(4)
    matrix[0, 0] = -1.0

    with pytest.raises(ValueError, match="not proper"):
        matrix_to_pose(matrix)


def test_valid_composition_remains_unchanged():
    parent = valid_pose()
    parent.position.y = 0.0
    parent.position.z = 0.0

    child = Pose()
    child.position.x = 2.0
    half = math.radians(90.0) * 0.5
    child.orientation.z = math.sin(half)
    child.orientation.w = math.cos(half)

    result = compose_poses(parent, child)

    assert result.position.x == pytest.approx(3.0)
    assert result.position.y == pytest.approx(0.0)
    assert result.position.z == pytest.approx(0.0)
    assert result.orientation.z == pytest.approx(math.sqrt(0.5))
    assert result.orientation.w == pytest.approx(math.sqrt(0.5))


def test_pose_data_to_pose_normalizes_finite_non_unit_quaternion():
    data = PoseData(
        position=Vector3Data(x=1.0, y=2.0, z=3.0),
        orientation=QuaternionData(x=0.0, y=0.0, z=0.0, w=2.0),
    )

    pose = pose_data_to_pose(data)

    assert pose.position.x == pytest.approx(1.0)
    assert pose.position.y == pytest.approx(2.0)
    assert pose.position.z == pytest.approx(3.0)
    assert pose.orientation.x == pytest.approx(0.0)
    assert pose.orientation.y == pytest.approx(0.0)
    assert pose.orientation.z == pytest.approx(0.0)
    assert pose.orientation.w == pytest.approx(1.0)


def test_pose_data_to_pose_rejects_zero_quaternion():
    data = PoseData(
        position=Vector3Data.zero(),
        orientation=QuaternionData(x=0.0, y=0.0, z=0.0, w=0.0),
    )

    with pytest.raises(ValueError, match="Quaternion norm is zero"):
        pose_data_to_pose(data)
