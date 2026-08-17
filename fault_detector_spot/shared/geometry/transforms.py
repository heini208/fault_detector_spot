"""Pose conversion and transformation utilities."""

import math
from typing import Union

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_matrix,
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


PoseMessage = Union[Pose, PoseStamped]


def pose_data_to_pose(data: PoseData) -> Pose:
    """Convert finite serializable pose data into a normalized ROS pose."""
    if not isinstance(data, PoseData):
        raise TypeError("Expected PoseData")
    data.position.validate()

    quaternion = normalized_quaternion(
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    )

    pose = Pose()
    pose.position.x = float(data.position.x)
    pose.position.y = float(data.position.y)
    pose.position.z = float(data.position.z)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def pose_to_pose_data(pose: PoseMessage) -> PoseData:
    """Convert a ROS pose into serializable pose data."""
    pose_message = get_pose(pose)
    _validate_position(pose_message)
    quaternion = normalized_quaternion(
        pose_message.orientation.x,
        pose_message.orientation.y,
        pose_message.orientation.z,
        pose_message.orientation.w,
    )

    result = PoseData(
        position=Vector3Data(
            x=float(pose_message.position.x),
            y=float(pose_message.position.y),
            z=float(pose_message.position.z),
        ),
        orientation=QuaternionData(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )
    result.validate()
    return result


def get_pose(pose: PoseMessage) -> Pose:
    """Return the inner Pose from Pose or PoseStamped."""
    if isinstance(pose, PoseStamped):
        return pose.pose
    if isinstance(pose, Pose):
        return pose
    raise TypeError(
        "Expected geometry_msgs/Pose or geometry_msgs/PoseStamped"
    )


def normalized_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple:
    """Return a finite normalized quaternion."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Quaternion contains a non-finite value")

    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Quaternion norm is zero")

    return tuple(value / norm for value in values)


def normalize_pose_quaternion(pose: Pose) -> None:
    """Normalize a pose quaternion in place."""
    if not isinstance(pose, Pose):
        raise TypeError("Expected geometry_msgs/Pose")
    _validate_position(pose)
    quaternion = normalized_quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]


def _validate_position(pose: Pose) -> None:
    values = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Pose position contains a non-finite value")


def _pose_rotation(pose: PoseMessage):
    pose_message = get_pose(pose)
    _validate_position(pose_message)
    x, y, z, w = normalized_quaternion(
        pose_message.orientation.x,
        pose_message.orientation.y,
        pose_message.orientation.z,
        pose_message.orientation.w,
    )
    return rotation_from_quaternion(
        QuaternionData(x=x, y=y, z=z, w=w)
    )


def pose_to_matrix(pose: PoseMessage) -> np.ndarray:
    """Convert a finite ROS pose into a homogeneous transform matrix."""
    pose_message = get_pose(pose)
    _validate_position(pose_message)

    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = _pose_rotation(pose_message).as_matrix()
    matrix[0, 3] = float(pose_message.position.x)
    matrix[1, 3] = float(pose_message.position.y)
    matrix[2, 3] = float(pose_message.position.z)

    if not np.all(np.isfinite(matrix)):
        raise ValueError("Pose produced a non-finite transform matrix")
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert a finite homogeneous transform matrix into a ROS pose."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Transform matrix must have shape 4x4")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Transform matrix is not finite")
    if not np.allclose(
        matrix[3],
        np.array([0.0, 0.0, 0.0, 1.0]),
        atol=1e-9,
        rtol=0.0,
    ):
        raise ValueError("Transform matrix has an invalid homogeneous row")

    rotation = matrix[:3, :3]
    orthogonality = rotation.T @ rotation
    if not np.allclose(
        orthogonality,
        np.eye(3),
        atol=1e-6,
        rtol=0.0,
    ):
        raise ValueError("Transform rotation matrix is not orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not math.isfinite(determinant) or not math.isclose(
        determinant,
        1.0,
        abs_tol=1e-6,
    ):
        raise ValueError("Transform rotation matrix is not proper")

    quaternion = quaternion_from_matrix(rotation)

    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    pose.orientation.x = quaternion.x
    pose.orientation.y = quaternion.y
    pose.orientation.z = quaternion.z
    pose.orientation.w = quaternion.w
    return pose


def compose_poses(
    parent_to_intermediate: PoseMessage,
    intermediate_to_child: PoseMessage,
) -> Pose:
    """Compose two poses using frame-chain order."""
    result = (
        pose_to_matrix(parent_to_intermediate)
        @ pose_to_matrix(intermediate_to_child)
    )
    return matrix_to_pose(result)


def inverse_pose(pose: PoseMessage) -> Pose:
    """Return the inverse of a pose transform."""
    matrix = pose_to_matrix(pose)
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return matrix_to_pose(inverse)


def relative_pose(
    reference_pose: PoseMessage,
    target_pose: PoseMessage,
) -> Pose:
    """Express a target pose relative to a reference pose."""
    return compose_poses(
        inverse_pose(reference_pose),
        target_pose,
    )
