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
    """Convert serializable pose data into a ROS pose."""
    pose = Pose()

    pose.position.x = data.position.x
    pose.position.y = data.position.y
    pose.position.z = data.position.z

    pose.orientation.x = data.orientation.x
    pose.orientation.y = data.orientation.y
    pose.orientation.z = data.orientation.z
    pose.orientation.w = data.orientation.w

    normalize_pose_quaternion(pose)
    return pose


def pose_to_pose_data(pose: PoseMessage) -> PoseData:
    """Convert a ROS pose into serializable pose data."""
    pose_message = get_pose(pose)
    quaternion = normalized_quaternion(
        pose_message.orientation.x,
        pose_message.orientation.y,
        pose_message.orientation.z,
        pose_message.orientation.w,
    )

    return PoseData(
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
    """Return a normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)

    if norm < 1e-12:
        raise ValueError("Quaternion norm is zero")

    return (
        float(x / norm),
        float(y / norm),
        float(z / norm),
        float(w / norm),
    )


def normalize_pose_quaternion(pose: Pose) -> None:
    """Normalize a pose quaternion in place."""
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


def _pose_rotation(pose: PoseMessage):
    pose_message = get_pose(pose)
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
    """Convert a ROS pose into a homogeneous transform matrix."""
    pose_message = get_pose(pose)
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = _pose_rotation(pose_message).as_matrix()
    matrix[0, 3] = pose_message.position.x
    matrix[1, 3] = pose_message.position.y
    matrix[2, 3] = pose_message.position.z
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert a homogeneous transform matrix into a ROS pose."""
    matrix = np.asarray(matrix, dtype=float)

    if matrix.shape != (4, 4):
        raise ValueError("Transform matrix must have shape 4x4")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Transform matrix is not finite")

    quaternion = quaternion_from_matrix(matrix[:3, :3])

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
