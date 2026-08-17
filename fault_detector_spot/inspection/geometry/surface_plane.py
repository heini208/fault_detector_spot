"""Shared fitted surface-plane geometry."""

import math
from dataclasses import dataclass

import numpy as np

from fault_detector_spot.inspection.geometry.open3d_depth import require_open3d
from fault_detector_spot.inspection.model.models import Vector3Data


@dataclass(frozen=True)
class SurfacePlane:
    """One fitted plane expressed in a named coordinate frame."""

    point: Vector3Data
    normal: Vector3Data
    frame_id: str
    inlier_count: int
    sample_count: int
    inlier_ratio: float
    rmse_m: float

    def validate(self) -> None:
        self.point.validate()
        self.normal.validate()
        if not self.frame_id.strip():
            raise ValueError("Surface plane frame must not be empty")
        if self.inlier_count < 3:
            raise ValueError("Surface plane requires at least three inliers")
        if self.sample_count < self.inlier_count:
            raise ValueError("Surface plane sample count is inconsistent")
        if not math.isfinite(self.inlier_ratio):
            raise ValueError("Surface plane inlier ratio must be finite")
        if not 0.0 < self.inlier_ratio <= 1.0:
            raise ValueError("Surface plane inlier ratio must be within (0, 1]")
        if not math.isfinite(self.rmse_m) or self.rmse_m < 0.0:
            raise ValueError("Surface plane RMSE must be finite and non-negative")
        normal = self.normal_array()
        if not math.isclose(float(np.linalg.norm(normal)), 1.0, abs_tol=1e-6):
            raise ValueError("Surface plane normal must be normalized")

    def point_array(self) -> np.ndarray:
        return np.array(
            [self.point.x, self.point.y, self.point.z],
            dtype=float,
        )

    def normal_array(self) -> np.ndarray:
        return np.array(
            [self.normal.x, self.normal.y, self.normal.z],
            dtype=float,
        )

    def signed_distance(self, point: Vector3Data) -> float:
        point.validate()
        value = np.array([point.x, point.y, point.z], dtype=float)
        return float(np.dot(value - self.point_array(), self.normal_array()))

    def oriented_toward(self, target: Vector3Data) -> "SurfacePlane":
        target.validate()
        direction = np.array(
            [target.x, target.y, target.z],
            dtype=float,
        ) - self.point_array()
        if float(np.dot(self.normal_array(), direction)) >= 0.0:
            return self
        return SurfacePlane(
            point=self.point,
            normal=Vector3Data(
                x=-self.normal.x,
                y=-self.normal.y,
                z=-self.normal.z,
            ),
            frame_id=self.frame_id,
            inlier_count=self.inlier_count,
            sample_count=self.sample_count,
            inlier_ratio=self.inlier_ratio,
            rmse_m=self.rmse_m,
        )


def fit_surface_plane(
    points,
    frame_id: str,
    distance_threshold_m: float,
    minimum_inliers: int,
    minimum_inlier_ratio: float,
    ransac_iterations: int,
    minimum_tangent_spread_m: float,
) -> SurfacePlane:
    """Fit one robust Open3D RANSAC plane to 3D samples."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("Surface-plane samples must have shape Nx3")
    if len(points) < 3:
        raise ValueError("Surface-plane fitting requires at least three samples")
    if not np.all(np.isfinite(points)):
        raise ValueError("Surface-plane samples must be finite")
    if not frame_id.strip():
        raise ValueError("Surface-plane frame must not be empty")
    if not math.isfinite(distance_threshold_m) or distance_threshold_m <= 0.0:
        raise ValueError("Surface-plane distance threshold must be positive")
    if (
        isinstance(minimum_inliers, bool)
        or not isinstance(minimum_inliers, int)
        or minimum_inliers < 3
    ):
        raise ValueError("Surface-plane minimum inliers must be at least three")
    if (
        not math.isfinite(minimum_inlier_ratio)
        or not 0.0 < minimum_inlier_ratio <= 1.0
    ):
        raise ValueError("Surface-plane inlier ratio must be within (0, 1]")
    if (
        isinstance(ransac_iterations, bool)
        or not isinstance(ransac_iterations, int)
        or ransac_iterations <= 0
    ):
        raise ValueError("Surface-plane RANSAC iterations must be positive")
    if (
        not math.isfinite(minimum_tangent_spread_m)
        or minimum_tangent_spread_m <= 0.0
    ):
        raise ValueError("Surface-plane tangent spread must be positive")

    open3d = require_open3d()
    cloud = open3d.geometry.PointCloud(
        open3d.utility.Vector3dVector(points)
    )
    plane_model, inlier_indices = cloud.segment_plane(
        distance_threshold_m,
        3,
        ransac_iterations,
    )
    inlier_indices = np.asarray(inlier_indices, dtype=int)
    inlier_ratio = len(inlier_indices) / len(points)
    if (
        len(inlier_indices) < minimum_inliers
        or inlier_ratio < minimum_inlier_ratio
    ):
        raise ValueError(
            "Local depth is not planar enough: "
            f"RANSAC kept {len(inlier_indices)}/{len(points)} samples "
            f"({inlier_ratio:.0%})"
        )

    inliers = points[inlier_indices]
    _validate_tangent_spread(inliers, minimum_tangent_spread_m)

    normal = np.asarray(plane_model[:3], dtype=float)
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("Surface normal cannot be normalized")
    normal = normal / norm
    offset = float(plane_model[3]) / norm

    distances = inliers @ normal + offset
    rmse_m = math.sqrt(float(np.mean(distances ** 2)))
    if not math.isfinite(rmse_m):
        raise ValueError("Surface-plane error is not finite")

    centroid = inliers.mean(axis=0)
    point = centroid - normal * (float(np.dot(normal, centroid)) + offset)
    result = SurfacePlane(
        point=_vector(point),
        normal=_vector(normal),
        frame_id=frame_id,
        inlier_count=len(inliers),
        sample_count=len(points),
        inlier_ratio=float(inlier_ratio),
        rmse_m=rmse_m,
    )
    result.validate()
    return result


def _validate_tangent_spread(points, minimum_tangent_spread_m) -> None:
    centered = points - points.mean(axis=0)
    covariance = centered.T @ centered / len(points)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if not np.all(np.isfinite(eigenvalues)):
        raise ValueError("Surface-plane covariance is not finite")
    tangent_spread_m = math.sqrt(max(float(eigenvalues[1]), 0.0))
    if tangent_spread_m < minimum_tangent_spread_m:
        raise ValueError(
            "Depth neighborhood does not span a two-dimensional surface"
        )


def _vector(values) -> Vector3Data:
    return Vector3Data(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
    )


__all__ = ["SurfacePlane", "fit_surface_plane"]
