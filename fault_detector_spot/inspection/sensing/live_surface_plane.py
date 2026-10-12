"""Shared local surface-plane estimate from a live hand-depth frame."""

from dataclasses import replace

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.surface_normal import (
    SurfaceNormalEstimate,
    estimate_surface_normal,
)
from fault_detector_spot.inspection.geometry.surface_plane import (
    DEFAULT_DEPTH_PLANE_TOLERANCE_M,
)
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)


SURFACE_ORIENTATION_WINDOW_RADIUS_PX = 16
SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX = 32
SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT = 30
MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M = 0.290


def estimate_live_surface_normal(
    depth_image: Image,
    camera_info: CameraInfo,
    window_radius_px: int = SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
) -> SurfaceNormalEstimate:
    """Fit the visible lower-middle surface shared by arm surface operations."""
    if (
        isinstance(window_radius_px, bool)
        or not isinstance(window_radius_px, int)
        or window_radius_px <= 0
    ):
        raise ValueError("Surface orientation window radius must be positive")

    point_cloud = create_organized_depth_point_cloud(
        depth_image, camera_info,
    )
    # Approximate the surface near the probe below the camera. This
    # assumes the visible patch continues toward the hidden contact region.
    # Keep the fit window above the bottom edge; small images stay centered.
    height = int(depth_image.height)
    fit_radius = max(
        window_radius_px, SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
    )
    center = ImagePoint(
        u=int(depth_image.width) // 2,
        v=max(height // 2, min(3 * height // 4, height - 1 - fit_radius)),
    )
    # Choose the dominant local depth rather than trusting one possibly
    # isolated foreground/outlier pixel as the depth gate for the plane.
    rows, columns = np.nonzero(point_cloud.valid_mask)
    nearby = ((columns - center.u) ** 2 + (rows - center.v) ** 2
              <= window_radius_px ** 2)
    rows, columns = rows[nearby], columns[nearby]
    seed = center
    if len(rows):
        depths = point_cloud.depth_m[rows, columns]
        median_depth = float(np.median(depths))
        supported = np.abs(depths - median_depth) <= 0.05
        if np.count_nonzero(supported) < 0.60 * len(depths):
            raise ValueError("Ambiguous lower-middle depth: no dominant surface")
        candidates = np.flatnonzero(supported)
        nearest = candidates[np.argmin(
            (columns[candidates] - center.u) ** 2
            + (rows[candidates] - center.v) ** 2
        )]
        seed = ImagePoint(u=int(columns[nearest]), v=int(rows[nearest]))
    try:
        projected = project_reference_pixel(
            seed,
            depth_image,
            camera_info,
            search_radius_px=window_radius_px,
            point_cloud=point_cloud,
            rgb_size=(
                int(depth_image.width),
                int(depth_image.height),
            ),
        )
    except ValueError as exception:
        if "No valid depth within" not in str(exception):
            raise
        raise ValueError(
            "No valid registered hand depth was found in the lower-middle "
            f"{window_radius_px} px window. The surface may be too "
            "close or too far from the gripper depth camera."
        ) from exception

    camera_distance_m = float(projected.depth_m)
    if camera_distance_m < MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:
        raise ValueError(
            "Surface is too close for reliable orientation: "
            f"camera distance {camera_distance_m:.3f} m, minimum "
            f"{MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:.3f} m"
        )

    try:
        estimate = estimate_surface_normal(
            projected,
            depth_image,
            camera_info,
            neighborhood_radius_px=max(
                window_radius_px, SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
            ),
            maximum_neighborhood_radius_px=max(
                window_radius_px,
                SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
            ),
            minimum_sample_count=(
                SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT
            ),
            point_cloud=point_cloud,
            neighborhood_center_pixel=projected.sampled_pixel,
            maximum_plane_rmse_m=DEFAULT_DEPTH_PLANE_TOLERANCE_M,
            minimum_tangent_spread_m=0.003,
        )
        stamp = depth_image.header.stamp
        return replace(
            estimate,
            projected_point=replace(projected, requested_pixel=center,
                                    mapped_pixel=center),
            stamp_nanoseconds=int(stamp.sec) * 1000000000 + int(stamp.nanosec),
        )
    except ValueError as exception:
        raise ValueError(
            "Unable to fit a reliable front surface plane: "
            f"{exception}"
        ) from exception

