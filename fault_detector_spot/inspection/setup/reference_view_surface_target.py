"""Generate surface-facing target poses for a selected reference point."""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_matrix,
    quaternion_to_rpy as scipy_quaternion_to_rpy,
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    validate_surface_distance_pair,
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
    validate_surface_distance_pair(
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    )

    controlled_frame_pose_object.validate()
    projected_point = approach_direction.projected_point
    projected_point.point_camera.validate()
    approach_direction.direction_camera.validate()

    rotation_object_camera = rotation_from_quaternion(
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
        + rotation_object_camera.apply(surface_point_camera)
    )
    outward_object_array = _normalized_vector(
        rotation_object_camera.apply(outward_camera),
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
    """Convert one quaternion into roll, pitch, and yaw with SciPy."""
    return scipy_quaternion_to_rpy(quaternion)


def _surface_facing_orientation(
    outward_direction_object: np.ndarray,
) -> QuaternionData:
    local_x = _normalized_vector(
        -outward_direction_object,
        "Surface inward direction",
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
    return quaternion_from_matrix(rotation)


def _vector_array(vector: Vector3Data, label: str) -> np.ndarray:
    vector.validate()
    return _shape_array(
        [vector.x, vector.y, vector.z],
        label,
    )


def _shape_array(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain three finite values")
    return array


def _normalized_vector(values, label: str) -> np.ndarray:
    array = _shape_array(values, label)
    norm = float(np.linalg.norm(array))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError(f"{label} cannot be normalized")
    return array / norm


def _vector_data(values) -> Vector3Data:
    array = _shape_array(values, "Vector")
    return Vector3Data(
        x=float(array[0]),
        y=float(array[1]),
        z=float(array[2]),
    )
