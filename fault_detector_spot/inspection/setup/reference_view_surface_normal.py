"""Estimate a local surface normal from registered reference depth."""

import math
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.open3d_depth import (
    OrganizedDepthPointCloud,
    create_organized_depth_point_cloud,
    require_open3d,
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
    """Fit a robust local plane with Open3D RANSAC."""
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
            return _fit_surface_plane(
                projected_point,
                samples,
                radius,
                minimum_sample_count,
                maximum_plane_rmse_m,
                minimum_tangent_spread_m,
                minimum_plane_inlier_ratio,
                ransac_iterations,
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


def _fit_surface_plane(
    projected_point,
    samples,
    radius,
    minimum_sample_count,
    maximum_plane_rmse_m,
    minimum_tangent_spread_m,
    minimum_plane_inlier_ratio,
    ransac_iterations,
) -> ReferenceSurfaceNormal:
    open3d = require_open3d()
    points = np.asarray(samples, dtype=np.float64)
    point_cloud = open3d.geometry.PointCloud(
        open3d.utility.Vector3dVector(points)
    )
    plane_model, inlier_indices = point_cloud.segment_plane(
        maximum_plane_rmse_m,
        3,
        ransac_iterations,
    )
    inlier_indices = np.asarray(inlier_indices, dtype=int)
    minimum_inliers = max(minimum_sample_count, 3)
    inlier_ratio = len(inlier_indices) / len(points)
    if (
        len(inlier_indices) < minimum_inliers
        or inlier_ratio < minimum_plane_inlier_ratio
    ):
        raise ValueError(
            "Local depth is not planar enough: "
            f"RANSAC kept {len(inlier_indices)}/{len(points)} samples "
            f"({inlier_ratio:.0%})"
        )

    normal = np.asarray(plane_model[:3], dtype=float)
    normal_norm = float(np.linalg.norm(normal))
    if not math.isfinite(normal_norm) or normal_norm <= 0.0:
        raise ValueError("Surface normal cannot be normalized")
    normal = normal / normal_norm
    plane_offset = float(plane_model[3]) / normal_norm

    inlier_points = points[inlier_indices]
    _validate_tangent_spread(
        inlier_points,
        minimum_tangent_spread_m,
    )

    camera_direction = -np.asarray([
        projected_point.point_camera.x,
        projected_point.point_camera.y,
        projected_point.point_camera.z,
    ], dtype=float)
    if float(np.dot(normal, camera_direction)) < 0.0:
        normal = -normal
        plane_offset = -plane_offset

    distances = inlier_points @ normal + plane_offset
    plane_rmse_m = math.sqrt(float(np.mean(distances ** 2)))
    if not math.isfinite(plane_rmse_m):
        raise ValueError("Surface-plane error is not finite")
    if plane_rmse_m > maximum_plane_rmse_m:
        raise ValueError(
            "Local depth is not planar enough: "
            f"RMSE {plane_rmse_m:.4f} m exceeds "
            f"{maximum_plane_rmse_m:.4f} m using "
            f"{len(inlier_points)} RANSAC inliers"
        )

    result = ReferenceSurfaceNormal(
        projected_point=projected_point,
        normal_camera=Vector3Data(
            x=float(normal[0]),
            y=float(normal[1]),
            z=float(normal[2]),
        ),
        sample_count=len(inlier_points),
        plane_rmse_m=plane_rmse_m,
        neighborhood_radius_px=radius,
    )
    result.normal_camera.validate()
    return result


def _validate_tangent_spread(points, minimum_tangent_spread_m) -> None:
    centroid = points.mean(axis=0)
    centered = points - centroid
    covariance = centered.T @ centered / len(points)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if not np.all(np.isfinite(eigenvalues)):
        raise ValueError("Surface-plane covariance is not finite")
    tangent_spread_m = math.sqrt(max(float(eigenvalues[1]), 0.0))
    if tangent_spread_m < minimum_tangent_spread_m:
        raise ValueError(
            "Depth neighborhood does not span a two-dimensional surface"
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

