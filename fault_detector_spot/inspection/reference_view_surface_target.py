"""Generate surface-facing target poses for a selected reference point."""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .models import (
    MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from .reference_view_approach_direction import ReferenceApproachDirection


@dataclass(frozen=True)
class ReferenceSurfaceTarget:
    """Nominal sensor-tip target and aligned pre-approach poses."""

    surface_point_object: Vector3Data
    outward_direction_object: Vector3Data
    target_pose_object: PoseData
    aligned_preapproach_pose_object: PoseData
    target_surface_distance_m: float
    aligned_preapproach_distance_m: float
    direction_source: str


def resolve_reference_surface_target(
    approach_direction: ReferenceApproachDirection,
    controlled_frame_pose_object: PoseData,
    target_surface_distance_m: float,
    aligned_preapproach_distance_m: float,
) -> ReferenceSurfaceTarget:
    """Resolve object-frame sensor-tip poses from a surface point."""
    if approach_direction is None:
        raise ValueError("No surface orientation is available")
    _require_positive_finite(
        target_surface_distance_m,
        "Target surface distance",
    )
    _require_positive_finite(
        aligned_preapproach_distance_m,
        "Aligned pre-approach distance",
    )
    separation = (
        aligned_preapproach_distance_m - target_surface_distance_m
    )
    if (
        separation < MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M
        and not math.isclose(
            separation,
            MINIMUM_ALIGNED_PREAPPROACH_SEPARATION_M,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        raise ValueError(
            "Aligned pre-approach distance must be at least 0.05 m "
            "greater than the target surface distance"
        )

    controlled_frame_pose_object.validate()
    projected_point = approach_direction.projected_point
    projected_point.point_camera.validate()
    approach_direction.direction_camera.validate()

    rotation_object_camera = _quaternion_rotation_matrix(
        controlled_frame_pose_object.orientation
    )
    camera_origin_object = _vector_array(
        controlled_frame_pose_object.position,
        "Reference camera position",
    )
    surface_point_camera = _vector_array(
        projected_point.point_camera,
        "Selected surface point",
    )
    outward_camera = _vector_array(
        approach_direction.direction_camera,
        "Surface outward direction",
    )

    surface_point_object_array = (
        camera_origin_object
        + rotation_object_camera @ surface_point_camera
    )
    outward_object_array = _normalized_vector(
        rotation_object_camera @ outward_camera,
        "Object-frame surface outward direction",
    )
    target_position = (
        surface_point_object_array
        + outward_object_array * target_surface_distance_m
    )
    preapproach_position = (
        surface_point_object_array
        + outward_object_array * aligned_preapproach_distance_m
    )
    orientation = _surface_facing_orientation(outward_object_array)

    surface_point_object = _vector_data(surface_point_object_array)
    outward_direction_object = _vector_data(outward_object_array)
    target_pose = PoseData(
        position=_vector_data(target_position),
        orientation=orientation,
    )
    preapproach_pose = PoseData(
        position=_vector_data(preapproach_position),
        orientation=orientation,
    )

    surface_point_object.validate()
    outward_direction_object.validate()
    target_pose.validate()
    preapproach_pose.validate()

    return ReferenceSurfaceTarget(
        surface_point_object=surface_point_object,
        outward_direction_object=outward_direction_object,
        target_pose_object=target_pose,
        aligned_preapproach_pose_object=preapproach_pose,
        target_surface_distance_m=float(target_surface_distance_m),
        aligned_preapproach_distance_m=float(
            aligned_preapproach_distance_m
        ),
        direction_source=approach_direction.source,
    )


def quaternion_to_rpy(
    quaternion: QuaternionData,
) -> Tuple[float, float, float]:
    """Convert one normalized quaternion into roll, pitch, and yaw."""
    quaternion.validate()
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def _surface_facing_orientation(
    outward_direction_object: np.ndarray,
) -> QuaternionData:
    local_x = _normalized_vector(
        outward_direction_object,
        "Surface outward direction",
    )
    preferred_up = np.array([0.0, 0.0, 1.0], dtype=float)
    local_z = preferred_up - np.dot(preferred_up, local_x) * local_x
    if float(np.linalg.norm(local_z)) <= 1e-8:
        preferred_up = np.array([0.0, 1.0, 0.0], dtype=float)
        local_z = preferred_up - np.dot(preferred_up, local_x) * local_x
    local_z = _normalized_vector(local_z, "Surface target up axis")
    local_y = _normalized_vector(
        np.cross(local_z, local_x),
        "Surface target lateral axis",
    )
    local_z = _normalized_vector(
        np.cross(local_x, local_y),
        "Surface target up axis",
    )
    rotation = np.column_stack((local_x, local_y, local_z))
    return _rotation_matrix_to_quaternion(rotation)


def _rotation_matrix_to_quaternion(
    rotation: np.ndarray,
) -> QuaternionData:
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("Surface target rotation must be 3x3")
    if not np.all(np.isfinite(rotation)):
        raise ValueError("Surface target rotation is not finite")
    if not np.allclose(
        rotation @ rotation.T,
        np.eye(3),
        atol=1e-6,
    ):
        raise ValueError("Surface target rotation is not orthonormal")
    if not math.isclose(
        float(np.linalg.det(rotation)),
        1.0,
        abs_tol=1e-6,
    ):
        raise ValueError("Surface target rotation is not right-handed")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        ) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
        ) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale

    values = np.array([x, y, z, w], dtype=float)
    values = values / np.linalg.norm(values)
    result = QuaternionData(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
        w=float(values[3]),
    )
    result.validate()
    return result


def _quaternion_rotation_matrix(
    quaternion: QuaternionData,
) -> np.ndarray:
    quaternion.validate()
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def _vector_array(vector: Vector3Data, label: str) -> np.ndarray:
    vector.validate()
    return _normalized_shape_array(
        [vector.x, vector.y, vector.z],
        label,
    )


def _normalized_shape_array(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain three finite values")
    return array


def _normalized_vector(values, label: str) -> np.ndarray:
    array = _normalized_shape_array(values, label)
    norm = float(np.linalg.norm(array))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError(f"{label} cannot be normalized")
    return array / norm


def _vector_data(values) -> Vector3Data:
    array = _normalized_shape_array(values, "Vector")
    return Vector3Data(
        x=float(array[0]),
        y=float(array[1]),
        z=float(array[2]),
    )


def _require_positive_finite(value: float, label: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
