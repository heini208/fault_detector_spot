"""Measure the saved probe target only where registered depth supports it."""

import math

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    camera_intrinsics,
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.pose import relative_pose
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    project_camera_point,
    target_camera_point,
    transform_surface_plane,
)
from fault_detector_spot.inspection.geometry.surface_normal import estimate_surface_normal
from fault_detector_spot.inspection.geometry.surface_plane import DEFAULT_DEPTH_PLANE_TOLERANCE_M
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.sensing.live_surface_distance import SurfaceDistanceSample
from fault_detector_spot.inspection.sensing.live_surface_plane import (
    MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M,
    SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
    SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT,
    SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
    project_reference_pixel,
)
from fault_detector_spot.shared.geometry.models import Vector3Data


# This is a correspondence bound, not a distance accuracy allowance. A plane
# near the mount must not be mistaken for the intended pre-approach wall.
MAXIMUM_TARGET_DISTANCE_MISMATCH_M = 0.10
TARGET_SUPPORT_RADIUS_PX = 3
MINIMUM_TARGET_SUPPORT_RATIO = 0.60
MINIMUM_TARGET_SUPPORT_SAMPLES = 4


def measure_sensor_tip_distance(
    depth_image, camera_info, reference_probe_pose, camera_pose_odom,
    expected_distance_m,
):
    """Return a plane and perpendicular distance in the ORIGINAL probe frame.

    Every fit is centered on the saved target's projection. Both that target
    patch and the eventual probe-axis intersection must have local depth
    support; nearby visible surfaces cannot replace an unseen contact region.
    """
    frame_id = depth_frame_id(depth_image, camera_info)
    point_camera = target_camera_point(
        reference_probe_pose, camera_pose_odom, expected_distance_m,
    )
    intrinsics = camera_intrinsics(
        camera_info, (depth_image.width, depth_image.height), "Depth CameraInfo",
    )
    u, v = project_camera_point(point_camera, intrinsics)
    center = ImagePoint(u=int(round(u)), v=int(round(v)))
    cloud = create_organized_depth_point_cloud(depth_image, camera_info)
    _target_patch(cloud, center, projected_pixel=(u, v))
    projected = project_reference_pixel(
        center, depth_image, camera_info,
        search_radius_px=TARGET_SUPPORT_RADIUS_PX,
        point_cloud=cloud,
        rgb_size=(int(depth_image.width), int(depth_image.height)),
    )
    if projected.depth_m < MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:
        raise ValueError(
            "Sensor-tip target is too close for reliable depth: "
            f"camera distance {projected.depth_m:.3f} m, minimum "
            f"{MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:.3f} m"
        )
    fit_radius = SURFACE_ORIENTATION_WINDOW_RADIUS_PX
    while True:
        estimate = estimate_surface_normal(
            projected, depth_image, camera_info,
            neighborhood_radius_px=fit_radius,
            maximum_neighborhood_radius_px=SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
            minimum_sample_count=SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT,
            maximum_plane_rmse_m=DEFAULT_DEPTH_PLANE_TOLERANCE_M,
            minimum_tangent_spread_m=0.003,
            point_cloud=cloud,
            neighborhood_center_pixel=center,
        )
        camera_plane = estimate.surface_plane_camera
        plane = transform_surface_plane(
            camera_plane, relative_pose(reference_probe_pose, camera_pose_odom), "probe",
        ).oriented_toward(Vector3Data.zero())
        distance = plane.signed_distance(Vector3Data.zero())
        if not math.isfinite(distance) or plane.normal.x >= -1e-6:
            raise ValueError("Measured target plane does not intersect forward probe +X")
        axis_distance = -distance / plane.normal.x
        if not 0.003 <= axis_distance <= 0.50:
            raise ValueError(
                f"Measured sensor-tip target is outside the 0.003–0.500 m range: {axis_distance:.3f} m"
            )
        if abs(axis_distance - expected_distance_m) > MAXIMUM_TARGET_DISTANCE_MISMATCH_M:
            raise ValueError(
                "Measured surface does not match the intended sensor-tip target: "
                f"axis distance {axis_distance:.3f} m, expected {expected_distance_m:.3f} m; "
                f"maximum difference {MAXIMUM_TARGET_DISTANCE_MISMATCH_M:.3f} m"
            )

        intersection_camera = target_camera_point(
            reference_probe_pose, camera_pose_odom, axis_distance,
        )
        iu, iv = project_camera_point(intersection_camera, intrinsics)
        intersection_pixel = ImagePoint(u=int(round(iu)), v=int(round(iv)))
        radius = estimate.neighborhood_radius_px
        required_radius = math.ceil(
            math.hypot(iu - center.u, iv - center.v) + TARGET_SUPPORT_RADIUS_PX,
        )
        if required_radius <= radius:
            break
        if required_radius > SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX:
            raise ValueError("Measured probe-axis intersection lies outside the observed target patch")
        # A camera view away from the optical center has parallax when the
        # measured wall differs from the nominal target distance. Refit the
        # actual supported region; never extrapolate the initial smaller fit.
        fit_radius = required_radius
    _target_patch(cloud, center, projected_pixel=(u, v), camera_plane=camera_plane)
    support_ratio = _target_patch(
        cloud, intersection_pixel, projected_pixel=(iu, iv), camera_plane=camera_plane,
    )
    x0, y0 = max(0, center.u - radius), max(0, center.v - radius)
    x1 = min(int(depth_image.width), center.u + radius + 1)
    y1 = min(int(depth_image.height), center.v + radius + 1)
    stamp = depth_image.header.stamp
    return SurfaceDistanceSample(
        distance_m=distance,
        stamp_seconds=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        frame_id=frame_id,
        sample_count=camera_plane.inlier_count,
        valid_pixel_ratio=support_ratio,
        spread_m=camera_plane.rmse_m,
        source_region=ImageRegion(x=x0, y=y0, width=x1-x0, height=y1-y0),
        surface_plane_probe=plane,
    )


def depth_frame_id(depth_image, camera_info):
    image_frame = depth_image.header.frame_id.strip()
    info_frame = camera_info.header.frame_id.strip()
    if image_frame and info_frame and image_frame != info_frame:
        raise ValueError("Live depth image and CameraInfo frames differ")
    frame_id = image_frame or info_frame
    if not frame_id:
        raise ValueError("Live depth frame is empty")
    return frame_id


def _target_patch(cloud, center, *, projected_pixel=None, camera_plane=None):
    """Require measured samples around a target, not a dense registered raster.

    Registration can leave regular empty pixels between valid depth returns.
    Four non-collinear returns enclosing the target protect local support;
    the larger fit still requires its own minimum sample count and spread.
    A plane must agree with the measured returns and cannot hide foreground.
    """
    radius = TARGET_SUPPORT_RADIUS_PX
    if not (
        radius <= center.u < cloud.width - radius
        and radius <= center.v < cloud.height - radius
    ):
        raise ValueError("Intended sensor-tip target patch is outside the camera image")
    rows = slice(center.v - radius, center.v + radius + 1)
    columns = slice(center.u - radius, center.u + radius + 1)
    mask = cloud.valid_mask[rows, columns].copy()
    valid_depth_count = int(np.count_nonzero(mask))
    detail = f"pixel ({center.u}, {center.v}), valid depth={valid_depth_count}/{mask.size}"
    if valid_depth_count < MINIMUM_TARGET_SUPPORT_SAMPLES:
        raise ValueError(
            "Intended sensor-tip target patch has insufficient unoccluded depth support: "
            f"need at least {MINIMUM_TARGET_SUPPORT_SAMPLES} measured samples; {detail}"
        )
    if camera_plane is not None:
        points = cloud.points_camera[rows, columns]
        rays = np.divide(
            points, points[..., 2, None], out=np.full_like(points, np.nan),
            where=mask[..., None],
        )
        denominator = np.einsum("ijk,k->ij", rays, camera_plane.normal_array())
        predicted = np.divide(
            np.dot(camera_plane.point_array(), camera_plane.normal_array()),
            denominator,
            out=np.full(mask.shape, np.nan),
            where=np.abs(denominator) > 1e-12,
        )
        residual = points[..., 2] - predicted
        if np.any(mask & (residual < -DEFAULT_DEPTH_PLANE_TOLERANCE_M)):
            raise ValueError(
                "Intended sensor-tip target patch has foreground depth closer than the fitted plane: "
                + detail
            )
        mask &= np.abs(residual) <= DEFAULT_DEPTH_PLANE_TOLERANCE_M
    support_count = int(np.count_nonzero(mask))
    # Density of a reprojected image says nothing about the fraction of valid
    # physical returns. Only actual returns enter the consistency denominator.
    ratio = float(support_count / valid_depth_count)
    if support_count < MINIMUM_TARGET_SUPPORT_SAMPLES or ratio < MINIMUM_TARGET_SUPPORT_RATIO:
        raise ValueError(
            "Intended sensor-tip target patch has insufficient unoccluded depth support: "
            f"plane-consistent depth={support_count}/{valid_depth_count} measured samples; {detail}"
        )

    local_rows, local_columns = np.nonzero(mask)
    u, v = projected_pixel if projected_pixel is not None else (center.u, center.v)
    offsets = np.column_stack((
        local_columns + center.u - radius - u,
        local_rows + center.v - radius - v,
    ))
    try:
        hull = ConvexHull(offsets)
    except QhullError as error:
        raise ValueError(
            "Measured depth does not surround the intended sensor-tip target: "
            f"support is collinear; {detail}"
        ) from error
    if np.any(hull.equations[:, -1] > 1e-9):
        raise ValueError(
            "Measured depth does not surround the intended sensor-tip target: "
            f"target is outside measured support; {detail}"
        )
    return ratio
