"""Estimate a local surface normal from registered reference depth."""

import math
from dataclasses import dataclass
from typing import List

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from .models import ImagePoint, Vector3Data
from .reference_view_depth_projection import (
    ProjectedReferencePoint,
    project_reference_pixel,
)


@dataclass(frozen=True)
class ReferenceSurfaceNormal:
    """Local plane normal associated with one projected surface point."""

    projected_point: ProjectedReferencePoint
    normal_camera: Vector3Data
    sample_count: int
    plane_rmse_m: float


def estimate_reference_surface_normal(
    projected_point: ProjectedReferencePoint,
    depth_image: Image,
    camera_info: CameraInfo,
    neighborhood_radius_px: int = 4,
    minimum_sample_count: int = 12,
    maximum_depth_delta_m: float = 0.03,
    maximum_plane_rmse_m: float = 0.008,
    minimum_tangent_spread_m: float = 0.0005,
) -> ReferenceSurfaceNormal:
    """Fit a local plane and return its camera-facing unit normal."""
    _validate_inputs(
        projected_point,
        neighborhood_radius_px,
        minimum_sample_count,
        maximum_depth_delta_m,
        maximum_plane_rmse_m,
        minimum_tangent_spread_m,
    )

    samples = _collect_surface_samples(
        projected_point,
        depth_image,
        camera_info,
        neighborhood_radius_px,
        maximum_depth_delta_m,
    )
    if len(samples) < minimum_sample_count:
        raise ValueError(
            "Too few consistent depth samples for surface-normal "
            f"estimation: {len(samples)} < {minimum_sample_count}"
        )

    points = np.asarray(samples, dtype=float)
    centroid = points.mean(axis=0)
    centered = points - centroid
    covariance = centered.T @ centered / len(points)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)

    if not np.all(np.isfinite(eigenvalues)):
        raise ValueError("Surface-plane covariance is not finite")

    tangent_spread_m = math.sqrt(max(float(eigenvalues[1]), 0.0))
    if tangent_spread_m < minimum_tangent_spread_m:
        raise ValueError(
            "Depth neighborhood does not span a two-dimensional surface"
        )

    normal = eigenvectors[:, 0]
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("Surface normal cannot be normalized")
    normal = normal / norm

    camera_direction = -centroid
    if float(np.dot(normal, camera_direction)) < 0.0:
        normal = -normal

    distances = centered @ normal
    plane_rmse_m = math.sqrt(float(np.mean(distances ** 2)))
    if not math.isfinite(plane_rmse_m):
        raise ValueError("Surface-plane error is not finite")
    if plane_rmse_m > maximum_plane_rmse_m:
        raise ValueError(
            "Local depth is not planar enough: "
            f"RMSE {plane_rmse_m:.4f} m exceeds "
            f"{maximum_plane_rmse_m:.4f} m"
        )

    result = ReferenceSurfaceNormal(
        projected_point=projected_point,
        normal_camera=Vector3Data(
            x=float(normal[0]),
            y=float(normal[1]),
            z=float(normal[2]),
        ),
        sample_count=len(points),
        plane_rmse_m=plane_rmse_m,
    )
    result.normal_camera.validate()
    return result


def _validate_inputs(
    projected_point,
    neighborhood_radius_px,
    minimum_sample_count,
    maximum_depth_delta_m,
    maximum_plane_rmse_m,
    minimum_tangent_spread_m,
) -> None:
    if projected_point is None:
        raise ValueError("No projected surface point is available")
    projected_point.requested_pixel.validate()
    projected_point.point_camera.validate()
    _require_non_negative_integer(
        neighborhood_radius_px,
        "Surface neighborhood radius",
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


def _collect_surface_samples(
    projected_point,
    depth_image,
    camera_info,
    radius,
    maximum_depth_delta_m,
) -> List[List[float]]:
    requested = projected_point.requested_pixel
    samples = []
    for v in range(
        max(0, requested.v - radius),
        min(depth_image.height, requested.v + radius + 1),
    ):
        for u in range(
            max(0, requested.u - radius),
            min(depth_image.width, requested.u + radius + 1),
        ):
            if (u - requested.u) ** 2 + (v - requested.v) ** 2 > radius ** 2:
                continue
            try:
                candidate = project_reference_pixel(
                    ImagePoint(u=u, v=v),
                    depth_image,
                    camera_info,
                    search_radius_px=0,
                )
            except ValueError:
                continue
            if abs(candidate.depth_m - projected_point.depth_m) > (
                maximum_depth_delta_m
            ):
                continue
            point = candidate.point_camera
            samples.append([point.x, point.y, point.z])
    return samples


def _require_non_negative_integer(value, label) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")


def _require_positive_integer(value, label) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")


def _require_positive_finite(value, label) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
