"""Resolve an outward approach direction for a selected reference point."""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .models import PoseData, Vector3Data
from .reference_view_depth_projection import ProjectedReferencePoint
from .reference_view_surface_normal import ReferenceSurfaceNormal


APPROACH_MODE_AUTOMATIC = "automatic"
APPROACH_MODE_SURFACE_FIT = "surface_fit"
APPROACH_MODE_TAG_X = "tag_x"

APPROACH_SOURCE_SURFACE_FIT = "surface_fit"
APPROACH_SOURCE_TAG_X_SELECTED = "tag_x_selected"

_VALID_MODES = {
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
}


@dataclass(frozen=True)
class ReferenceApproachDirection:
    """Camera-frame direction pointing outward from the inspected object."""

    projected_point: ProjectedReferencePoint
    direction_camera: Vector3Data
    source: str
    surface_normal: Optional[ReferenceSurfaceNormal] = None


def resolve_reference_approach_direction(
    projected_point: ProjectedReferencePoint,
    surface_normal: Optional[ReferenceSurfaceNormal],
    controlled_frame_pose_object: PoseData,
    mode: str = APPROACH_MODE_AUTOMATIC,
    surface_normal_unavailable_reason: str = "",
) -> ReferenceApproachDirection:
    """Resolve an outward direction from geometry or an explicit tag axis."""
    if projected_point is None:
        raise ValueError("No projected surface point is available")
    projected_point.requested_pixel.validate()
    projected_point.point_camera.validate()
    if mode not in _VALID_MODES:
        raise ValueError(f"Unsupported approach-direction mode: {mode}")

    tag_x_camera = _object_positive_x_in_camera(
        controlled_frame_pose_object
    )

    if mode == APPROACH_MODE_TAG_X:
        return _make_result(
            projected_point,
            tag_x_camera,
            APPROACH_SOURCE_TAG_X_SELECTED,
        )

    if surface_normal is not None:
        direction = _aligned_surface_direction(
            projected_point,
            surface_normal,
        )
        return _make_result(
            projected_point,
            direction,
            APPROACH_SOURCE_SURFACE_FIT,
            surface_normal=surface_normal,
        )

    reason = surface_normal_unavailable_reason.strip()
    if not reason:
        reason = "No valid local surface normal is available"
    raise ValueError(
        f"Surface-fit approach direction is unavailable: {reason}"
    )


def _object_positive_x_in_camera(
    controlled_frame_pose_object: PoseData,
) -> np.ndarray:
    controlled_frame_pose_object.validate()
    orientation = controlled_frame_pose_object.orientation
    rotation_object_camera = _quaternion_rotation_matrix(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    direction = rotation_object_camera.T @ np.array(
        [1.0, 0.0, 0.0],
        dtype=float,
    )
    return _normalized_vector(direction, "Tag-frame +X direction")


def _aligned_surface_direction(
    projected_point: ProjectedReferencePoint,
    surface_normal: ReferenceSurfaceNormal,
) -> np.ndarray:
    normal_point = surface_normal.projected_point
    if normal_point.frame_id != projected_point.frame_id:
        raise ValueError(
            "Surface normal and projected point frames do not match"
        )
    if normal_point.requested_pixel != projected_point.requested_pixel:
        raise ValueError(
            "Surface normal belongs to a different reference pixel"
        )

    normal = surface_normal.normal_camera
    normal.validate()
    direction = _normalized_vector(
        np.array([normal.x, normal.y, normal.z], dtype=float),
        "Surface normal",
    )
    camera_facing = _normalized_vector(
        -np.array([
            projected_point.point_camera.x,
            projected_point.point_camera.y,
            projected_point.point_camera.z,
        ]),
        "Camera-facing surface direction",
    )
    if float(np.dot(direction, camera_facing)) < 0.0:
        direction = -direction
    return direction


def _make_result(
    projected_point,
    direction,
    source,
    surface_normal=None,
) -> ReferenceApproachDirection:
    normalized = _normalized_vector(direction, "Approach direction")
    result = ReferenceApproachDirection(
        projected_point=projected_point,
        direction_camera=Vector3Data(
            x=float(normalized[0]),
            y=float(normalized[1]),
            z=float(normalized[2]),
        ),
        source=source,
        surface_normal=surface_normal,
    )
    result.direction_camera.validate()
    return result


def _normalized_vector(vector, label: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must contain three finite values")
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError(f"{label} cannot be normalized")
    return vector / norm


def _quaternion_rotation_matrix(x, y, z, w) -> np.ndarray:
    values = np.asarray([x, y, z, w], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("Reference-view orientation is not finite")
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Reference-view orientation has zero norm")
    x, y, z, w = values / norm

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
