"""Estimate a local surface normal from registered reference depth."""

import math
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.open3d_depth import (
    OrganizedDepthPointCloud,
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.surface_plane import (
    fit_surface_plane,
)
from fault_detector_spot.inspection.model.models import Vector3Data
from .reference_view_depth_projection import ProjectedReferencePoint


@dataclass(frozen=True)
class ReferenceSurfaceNormal:
    """Local plane normal associated with one projected surface point."""

    projected_point: ProjectedReferencePoint
    normal_camera: Vector3Data
    sample_count: int
    plane_rmse_m: float
    neighborhood_radius_px: int = 0


def estimate_reference_surface_normal(
    projected_point: ProjectedReferencePoint,
    depth_image: Image,
    camera_info: CameraInfo,
    neighborhood_radius_px: int = 4,
    maximum_neighborhood_radius_px: int = 12,
    minimum_sample_count: int = 12,
    maximum_depth_delta_m: float = 0.05,
    maximum_plane_rmse_m: float = 0.015,
    minimum_tangent_spread_m: float = 0.0005,
    minimum_plane_inlier_ratio: float = 0.60,
    ransac_iterations: int = 100,
) -> ReferenceSurfaceNormal:
    """Fit a robust local plane with shared Open3D geometry."""
    _validate_inputs(
        projected_point,
        neighborhood_radius_px,
        maximum_neighborhood_radius_px,
        minimum_sample_count,
        maximum_depth_delta_m,
        maximum_plane_rmse_m,
        minimum_tangent_spread_m,
        minimum_plane_inlier_ratio,
        ransac_iterations,
    )

    point_cloud = create_organized_depth_point_cloud(
        depth_image,
        camera_info,
    )
    best_sample_count = 0
    last_error = None
    for radius in _candidate_radii(
        neighborhood_radius_px,
        maximum_neighborhood_radius_px,
    ):
        samples = _collect_surface_samples(
            projected_point,
            point_cloud,
            radius,
            maximum_depth_delta_m,
        )
        best_sample_count = max(best_sample_count, len(samples))
        if len(samples) < max(minimum_sample_count, 3):
            continue
        try:
            plane = fit_surface_plane(
                samples,
                projected_point.frame_id,
                maximum_plane_rmse_m,
                max(minimum_sample_count, 3),
                minimum_plane_inlier_ratio,
                ransac_iterations,
                minimum_tangent_spread_m,
            ).oriented_toward(Vector3Data(x=0.0, y=0.0, z=0.0))
            if plane.rmse_m > maximum_plane_rmse_m:
                raise ValueError(
                    "Local depth is not planar enough: "
                    f"RMSE {plane.rmse_m:.4f} m exceeds "
                    f"{maximum_plane_rmse_m:.4f} m using "
                    f"{plane.inlier_count} RANSAC inliers"
                )
            return ReferenceSurfaceNormal(
                projected_point=projected_point,
                normal_camera=plane.normal,
                sample_count=plane.inlier_count,
                plane_rmse_m=plane.rmse_m,
                neighborhood_radius_px=radius,
            )
        except ValueError as exception:
            last_error = exception

    if last_error is not None:
        raise last_error
    raise ValueError(
        "Too few consistent depth samples for surface-normal "
        f"estimation: {best_sample_count} < {minimum_sample_count}; "
        f"searched through radius {maximum_neighborhood_radius_px} px"
    )


def _candidate_radii(initial_radius: int, maximum_radius: int):
    radius = initial_radius
    while radius < maximum_radius:
        yield radius
        radius += 2
    yield maximum_radius


def _validate_inputs(
    projected_point,
    neighborhood_radius_px,
    maximum_neighborhood_radius_px,
    minimum_sample_count,
    maximum_depth_delta_m,
    maximum_plane_rmse_m,
    minimum_tangent_spread_m,
    minimum_plane_inlier_ratio,
    ransac_iterations,
) -> None:
    if projected_point is None:
        raise ValueError("No projected surface point is available")
    projected_point.requested_pixel.validate()
    projected_point.mapped_pixel.validate()
    projected_point.sampled_pixel.validate()
    projected_point.point_camera.validate()
    _require_non_negative_integer(
        neighborhood_radius_px,
        "Surface neighborhood radius",
    )
    _require_non_negative_integer(
        maximum_neighborhood_radius_px,
        "Maximum surface neighborhood radius",
    )
    if maximum_neighborhood_radius_px < neighborhood_radius_px:
        raise ValueError(
            "Maximum surface neighborhood radius must not be smaller "
            "than the initial radius"
        )
    _require_positive_integer(
        minimum_sample_count,
        "Minimum surface sample count",
    )
    _require_positive_finite(
        maximum_depth_delta_m,
        "Maximum surface depth delta",
    )
    _require_positive_finite(
        maximum_plane_rmse_m,
        "Maximum surface-plane RMSE",
    )
    _require_positive_finite(
        minimum_tangent_spread_m,
        "Minimum tangent spread",
    )
    if (
        not math.isfinite(minimum_plane_inlier_ratio)
        or minimum_plane_inlier_ratio <= 0.0
        or minimum_plane_inlier_ratio > 1.0
    ):
        raise ValueError(
            "Minimum plane inlier ratio must be within (0, 1]"
        )
    _require_positive_integer(
        ransac_iterations,
        "RANSAC iteration count",
    )


def _collect_surface_samples(
    projected_point,
    point_cloud: OrganizedDepthPointCloud,
    radius,
    maximum_depth_delta_m,
) -> np.ndarray:
    center = projected_point.mapped_pixel
    u_min = max(0, center.u - radius)
    u_max = min(point_cloud.width, center.u + radius + 1)
    v_min = max(0, center.v - radius)
    v_max = min(point_cloud.height, center.v + radius + 1)

    points = point_cloud.points_camera[v_min:v_max, u_min:u_max]
    depths = point_cloud.depth_m[v_min:v_max, u_min:u_max]
    valid = point_cloud.valid_mask[v_min:v_max, u_min:u_max].copy()
    u_grid, v_grid = np.meshgrid(
        np.arange(u_min, u_max),
        np.arange(v_min, v_max),
    )
    valid &= (
        (u_grid - center.u) ** 2
        + (v_grid - center.v) ** 2
        <= radius ** 2
    )
    valid &= (
        np.abs(depths - projected_point.depth_m)
        <= maximum_depth_delta_m
    )
    return np.asarray(points[valid], dtype=np.float64)


def _require_non_negative_integer(value, label) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")


def _require_positive_integer(value, label) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")


def _require_positive_finite(value, label) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
