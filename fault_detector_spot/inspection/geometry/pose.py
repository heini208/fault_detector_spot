"""PoseData rigid-transform helpers shared by setup and execution."""

import math

from fault_detector_spot.inspection.geometry.rotation import (
    inverse_quaternion,
    multiply_quaternions,
    rotate_vector,
)
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data


def compose_poses(
    parent_to_child: PoseData,
    child_to_target: PoseData,
) -> PoseData:
    parent_to_child.validate()
    child_to_target.validate()
    rotated_position = rotate_vector(
        parent_to_child.orientation,
        child_to_target.position,
    )
    result = PoseData(
        position=add_vectors(
            parent_to_child.position,
            rotated_position,
        ),
        orientation=multiply_quaternions(
            parent_to_child.orientation,
            child_to_target.orientation,
        ),
    )
    result.validate()
    return result


def relative_pose(
    parent_to_reference: PoseData,
    parent_to_target: PoseData,
) -> PoseData:
    return compose_poses(
        inverse_pose(parent_to_reference),
        parent_to_target,
    )


def inverse_pose(pose: PoseData) -> PoseData:
    pose.validate()
    inverse_orientation = inverse_quaternion(pose.orientation)
    inverse_position = rotate_vector(
        inverse_orientation,
        Vector3Data(
            x=-pose.position.x,
            y=-pose.position.y,
            z=-pose.position.z,
        ),
    )
    result = PoseData(
        position=inverse_position,
        orientation=inverse_orientation,
    )
    result.validate()
    return result


def probe_pose_to_hand_pose(
    desired_probe_pose_parent: PoseData,
    hand_to_probe_pose: PoseData,
) -> PoseData:
    desired_probe_pose_parent.validate()
    hand_to_probe_pose.validate()
    return compose_poses(
        desired_probe_pose_parent,
        inverse_pose(hand_to_probe_pose),
    )


def add_vectors(first: Vector3Data, second: Vector3Data) -> Vector3Data:
    first.validate()
    second.validate()
    return Vector3Data(
        x=first.x + second.x,
        y=first.y + second.y,
        z=first.z + second.z,
    )


def subtract_vectors(first: Vector3Data, second: Vector3Data) -> Vector3Data:
    first.validate()
    second.validate()
    return Vector3Data(
        x=first.x - second.x,
        y=first.y - second.y,
        z=first.z - second.z,
    )


def scale_vector(vector: Vector3Data, factor: float) -> Vector3Data:
    vector.validate()
    if not math.isfinite(factor):
        raise ValueError("Vector scale must be finite")
    return Vector3Data(
        x=vector.x * factor,
        y=vector.y * factor,
        z=vector.z * factor,
    )


__all__ = [
    "add_vectors",
    "compose_poses",
    "inverse_pose",
    "probe_pose_to_hand_pose",
    "relative_pose",
    "scale_vector",
    "subtract_vectors",
]
