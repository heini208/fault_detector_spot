"""Find the surface along a fixed probe axis using registered depth."""

import math

import numpy as np
from scipy.ndimage import correlate, minimum_filter
from scipy.spatial import ConvexHull, QhullError

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    camera_intrinsics,
    create_organized_depth_point_cloud,
    depth_below_minimum,
)
from fault_detector_spot.inspection.geometry.pose import relative_pose
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    observation_translations_camera,
    project_camera_point,
    probe_axis_in_camera,
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
    estimate_live_surface_normal,
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
MAXIMUM_OBSERVATION_PLANE_EXTENSION_M = 0.15
PROBE_AXIS_SEARCH_RADIUS_M = 0.025
MAXIMUM_SEED_FITS = 8
SEED_DEPTH_BAND_M = 0.05
OBSERVATION_TARGET_MARGIN_M = 0.003
MAXIMUM_OBSERVATION_PLANE_NORMAL_DIFFERENCE_DEG = 10.0


class SensorTipTargetOutsideView(ValueError):
    """A nearby plane can guide observation, but is not a target measurement."""

    def __init__(self, axis_distance_m, detail, *, observation_pixel=None, target_plane_probe=None):
        super().__init__(detail)
        self.axis_distance_m = axis_distance_m
        self.observation_pixel = observation_pixel
        self.target_plane_probe = target_plane_probe


class SensorTipTargetUnavailable(ValueError):
    """Local depth cannot establish the target, but another view may help."""


class _NonForwardProbePlane(ValueError):
    """A fitted side surface does not cross the forward probe ray."""


class _NearCameraSurface(SensorTipTargetUnavailable):
    """A coherent near surface cannot be measured reliably by this camera."""


class _ObservedTargetBlocked(SensorTipTargetUnavailable):
    """A nearer coherent surface blocks an already observed wall target."""


def measure_sensor_tip_distance(
    depth_image, camera_info, reference_probe_pose, camera_pose_odom,
    expected_distance_m=None, *, target_plane_probe=None,
):
    """Return a plane and perpendicular distance in the ORIGINAL probe frame.

    Discovery uses measured depth nearest the forward probe ray. Once an
    observation has identified a wall, its measured plane retains the target
    and normal association across the camera excursion. Fresh local depth
    must support both that target and the newly measured ray intersection;
    nearer coherent or on-ray geometry remains blocking. A saved goal may
    supply a distance correspondence check, but cannot select the target.
    """
    frame_id = depth_frame_id(depth_image, camera_info)
    if expected_distance_m is not None and (
        not math.isfinite(expected_distance_m) or expected_distance_m <= 0.0
    ):
        raise ValueError("Expected sensor-tip distance must be positive and finite")
    intrinsics = camera_intrinsics(
        camera_info, (depth_image.width, depth_image.height), "Depth CameraInfo",
    )
    cloud = create_organized_depth_point_cloud(depth_image, camera_info)
    if target_plane_probe is None:
        return _measure_sensor_tip_depth_cloud(
            depth_image, camera_info, reference_probe_pose, camera_pose_odom,
            expected_distance_m, frame_id, cloud, intrinsics,
        )
    target_plane_probe.validate()
    if target_plane_probe.frame_id != "probe" or target_plane_probe.normal.x >= -1e-6:
        raise ValueError("Observed sensor-tip plane must face forward probe +X in the probe frame")
    target_axis_distance_m = -target_plane_probe.signed_distance(Vector3Data.zero()) / target_plane_probe.normal.x
    if not math.isfinite(target_axis_distance_m) or not 0.003 <= target_axis_distance_m <= 0.50:
        raise ValueError("Observed sensor-tip target must lie in the 0.003–0.500 m range")
    target_camera = target_camera_point(
        reference_probe_pose, camera_pose_odom, target_axis_distance_m,
    )
    u, v = project_camera_point(target_camera, intrinsics)
    center = ImagePoint(u=int(round(u)), v=int(round(v)))
    try:
        _target_patch(cloud, center, projected_pixel=(u, v))
    except ValueError as error:
        raise SensorTipTargetUnavailable(str(error)) from error
    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose_odom)
    ray_pixels = _probe_ray_pixel_mask(cloud, reference_probe_pose, camera_pose_odom, intrinsics)
    rows, columns = np.nonzero(ray_pixels)
    points = cloud.points_camera[rows, columns]
    axial = np.einsum("ij,j->i", points - origin, direction)
    nearer = axial < target_axis_distance_m - DEFAULT_DEPTH_PLANE_TOLERANCE_M
    if np.any(nearer):
        index = np.flatnonzero(nearer)[np.argmin(axial[nearer])]
        raise SensorTipTargetUnavailable(
            "Measured foreground lies on probe +X before the observed target: "
            f"camera depth {points[index, 2]:.3f} m at pixel "
            f"({columns[index]}, {rows[index]})"
        )
    # Keep nearer coherent intersections authoritative, including unresolved
    # intersections outside the image. Only an off-axis extrapolation into
    # an unsupported in-image location may yield to the observed wall target.
    try:
        discovered = _measure_sensor_tip_depth_cloud(
            depth_image, camera_info, reference_probe_pose, camera_pose_odom,
            None, frame_id, cloud, intrinsics, blocking_axis_distance_m=target_axis_distance_m,
        )
    except (_NearCameraSurface, _ObservedTargetBlocked):
        raise
    except SensorTipTargetOutsideView as error:
        if error.axis_distance_m < target_axis_distance_m - DEFAULT_DEPTH_PLANE_TOLERANCE_M:
            raise
        discovered = None
    except ValueError:
        discovered = None
    if discovered is not None:
        discovered_axis = -discovered.distance_m / discovered.surface_plane_probe.normal.x
        if discovered_axis < target_axis_distance_m - DEFAULT_DEPTH_PLANE_TOLERANCE_M:
            raise SensorTipTargetUnavailable(
                "A locally supported nearer surface blocks the observed sensor-tip target: "
                f"probe-axis distance {discovered_axis:.3f} m, "
                f"observed target {target_axis_distance_m:.3f} m"
            )
    return _measure_sensor_tip_depth_cloud(
        depth_image, camera_info, reference_probe_pose, camera_pose_odom,
        expected_distance_m, frame_id, cloud, intrinsics, center=center,
        anchor_plane_probe=target_plane_probe,
    )


def _measure_sensor_tip_depth_cloud(
    depth_image, camera_info, reference_probe_pose, camera_pose_odom,
    expected_distance_m, frame_id, cloud, intrinsics, *, center=None,
    anchor_plane_probe=None, blocking_axis_distance_m=None,
):
    anchor_axis_distance_m = None if anchor_plane_probe is None else (
        -anchor_plane_probe.signed_distance(Vector3Data.zero()) / anchor_plane_probe.normal.x
    )
    if center is None:
        center = _nearest_axis_depth_pixel(
            cloud, reference_probe_pose, camera_pose_odom, intrinsics,
        )
    fit_radius = SURFACE_ORIENTATION_WINDOW_RADIUS_PX
    recentered = False
    excluded_seeds = np.zeros_like(cloud.valid_mask)
    ray_pixels = _probe_ray_pixel_mask(cloud, reference_probe_pose, camera_pose_odom, intrinsics)
    seed_failures = 0
    while True:
        projected = project_reference_pixel(
            center, depth_image, camera_info,
            search_radius_px=TARGET_SUPPORT_RADIUS_PX,
            point_cloud=cloud,
            rgb_size=(int(depth_image.width), int(depth_image.height)),
        )
        too_close = depth_below_minimum(projected.depth_m, MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M)
        near_error = (
            "Depth return near probe +X is too close for reliable measurement: "
            f"camera distance {projected.depth_m:.3f} m, minimum "
            f"{MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M:.3f} m "
            f"at pixel ({projected.sampled_pixel.u}, {projected.sampled_pixel.v})"
        )
        on_ray = ray_pixels[projected.sampled_pixel.v, projected.sampled_pixel.u]
        if too_close and on_ray:
            raise SensorTipTargetUnavailable(near_error)
        try:
            estimate = estimate_surface_normal(
                projected, depth_image, camera_info,
                neighborhood_radius_px=fit_radius,
                maximum_neighborhood_radius_px=SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
                minimum_sample_count=SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT,
                maximum_plane_rmse_m=DEFAULT_DEPTH_PLANE_TOLERANCE_M,
                # A thin coherent near surface is enough to BLOCK; it is
                # never accepted as a measurement below the camera limit.
                minimum_tangent_spread_m=0.0005 if too_close else 0.003,
                maximum_depth_delta_m=SEED_DEPTH_BAND_M,
                point_cloud=cloud,
                neighborhood_center_pixel=center,
            )
        except ValueError as error:
            seed_failures += 1
            if (anchor_axis_distance_m is not None or on_ray or recentered
                    or seed_failures >= MAXIMUM_SEED_FITS):
                raise SensorTipTargetUnavailable(near_error if too_close else str(error)) from error
            # A speck beside the ray cannot establish a surface. Discard
            # only unsupported seed candidates from this local depth band;
            # actual ray pixels and the raw target-support cloud remain.
            try:
                center = _next_axis_depth_seed(
                    cloud, reference_probe_pose, camera_pose_odom, intrinsics,
                    center, projected.depth_m, excluded_seeds, ray_pixels,
                )
            except ValueError:
                raise SensorTipTargetUnavailable(near_error if too_close else str(error)) from error
            continue
        if too_close:
            # Coherent nearer surfaces remain blocking even when their
            # inferred probe intersection lies outside the current image.
            raise _NearCameraSurface(near_error)
        if blocking_axis_distance_m is not None:
            _check_observed_target_blocker(
                cloud, estimate.surface_plane_camera, reference_probe_pose,
                camera_pose_odom, intrinsics, blocking_axis_distance_m,
            )
        if estimate.surface_plane_camera.signed_distance(projected.point_camera) > DEFAULT_DEPTH_PLANE_TOLERANCE_M:
            raise SensorTipTargetUnavailable("Closest probe-axis depth is foreground of the fitted plane")
        camera_plane = estimate.surface_plane_camera
        try:
            plane, distance, axis_distance = _probe_target_geometry(
                camera_plane, projected, reference_probe_pose, camera_pose_odom, expected_distance_m,
            )
        except _NonForwardProbePlane:
            seed_failures += 1
            if (anchor_axis_distance_m is not None or on_ray or recentered
                    or seed_failures >= MAXIMUM_SEED_FITS):
                raise
            # Side geometry can be planar and close to the probe without
            # crossing its forward ray. Continue only for that geometric
            # case; coherent forward surfaces and their support failures
            # remain terminal, and actual ray returns are never excluded.
            center = _next_axis_depth_seed(
                cloud, reference_probe_pose, camera_pose_odom, intrinsics,
                center, projected.depth_m, excluded_seeds, ray_pixels,
            )
            continue

        if anchor_axis_distance_m is not None:
            alignment = float(np.dot(plane.normal_array(), anchor_plane_probe.normal_array()))
            if alignment < math.cos(math.radians(MAXIMUM_OBSERVATION_PLANE_NORMAL_DIFFERENCE_DEG)):
                angle = math.degrees(math.acos(float(np.clip(alignment, -1.0, 1.0))))
                raise SensorTipTargetUnavailable(
                    "Measured surface normal does not agree with the observed sensor-tip target: "
                    f"difference {angle:.1f} deg, maximum "
                    f"{MAXIMUM_OBSERVATION_PLANE_NORMAL_DIFFERENCE_DEG:.1f} deg"
                )
            if abs(axis_distance-anchor_axis_distance_m) > DEFAULT_DEPTH_PLANE_TOLERANCE_M:
                raise SensorTipTargetUnavailable(
                    "Measured surface does not agree with the observed sensor-tip target: "
                    f"probe-axis distance {axis_distance:.3f} m, "
                    f"observed target {anchor_axis_distance_m:.3f} m"
                )
            anchor_camera = target_camera_point(
                reference_probe_pose, camera_pose_odom, anchor_axis_distance_m,
            )
            au, av = project_camera_point(anchor_camera, intrinsics)
            try:
                _target_patch(
                    cloud, center, projected_pixel=(au, av), camera_plane=camera_plane,
                )
            except ValueError as error:
                raise SensorTipTargetUnavailable(str(error)) from error

        intersection_camera = target_camera_point(
            reference_probe_pose, camera_pose_odom, axis_distance,
        )
        iu, iv = project_camera_point(intersection_camera, intrinsics)
        intersection_pixel = ImagePoint(u=int(round(iu)), v=int(round(iv)))
        margin = TARGET_SUPPORT_RADIUS_PX
        if not (margin <= intersection_pixel.u < cloud.width - margin
                and margin <= intersection_pixel.v < cloud.height - margin):
            raise SensorTipTargetOutsideView(
                axis_distance,
                "Measured probe-axis target patch is outside the camera image",
                target_plane_probe=plane,
            )
        try:
            support_ratio = _target_patch(
                cloud, intersection_pixel, projected_pixel=(iu, iv), camera_plane=camera_plane,
            )
        except ValueError as error:
            raise SensorTipTargetUnavailable(str(error)) from error
        radius = estimate.neighborhood_radius_px
        required_radius = math.ceil(
            math.hypot(iu - center.u, iv - center.v) + TARGET_SUPPORT_RADIUS_PX,
        )
        if required_radius <= radius:
            break
        if required_radius > SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX:
            if not recentered and anchor_axis_distance_m is None:
                # Actual depth was checked above. Refit there rather than
                # reporting an extrapolated normal from a neighboring patch.
                center = intersection_pixel
                fit_radius = SURFACE_ORIENTATION_WINDOW_RADIUS_PX
                recentered = True
                continue
            raise SensorTipTargetUnavailable("Measured probe-axis intersection lies outside the observed target patch")
        # Include the actual supported intersection in the local plane fit.
        fit_radius = required_radius
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


def _check_observed_target_blocker(
    cloud, camera_plane, reference_probe_pose, camera_pose, intrinsics, target_axis_distance_m,
):
    """Preserve established nearer geometry before considering a saved wall.

    A side-plane extrapolation into an unsupported in-image location is
    not an observed obstruction. A nearer intersection outside the image
    remains unresolved, even when its normal differs from the saved wall.
    """
    plane = transform_surface_plane(
        camera_plane, relative_pose(reference_probe_pose, camera_pose), "probe",
    ).oriented_toward(Vector3Data.zero())
    if plane.normal.x >= -1e-6:
        return
    axis_distance = -plane.signed_distance(Vector3Data.zero()) / plane.normal.x
    if not 0.003 <= axis_distance < target_axis_distance_m - DEFAULT_DEPTH_PLANE_TOLERANCE_M:
        return
    point = target_camera_point(reference_probe_pose, camera_pose, axis_distance)
    u, v = project_camera_point(point, intrinsics)
    pixel = ImagePoint(u=int(round(u)), v=int(round(v)))
    radius = TARGET_SUPPORT_RADIUS_PX
    if not (radius <= pixel.u < cloud.width-radius and radius <= pixel.v < cloud.height-radius):
        raise SensorTipTargetOutsideView(
            axis_distance,
            "A coherent nearer probe-axis surface has its target patch outside the camera image",
            target_plane_probe=plane,
        )
    try:
        _target_patch(cloud, pixel, projected_pixel=(u, v), camera_plane=camera_plane)
    except ValueError:
        return
    raise _ObservedTargetBlocked(
        "A locally supported nearer surface blocks the observed sensor-tip target: "
        f"probe-axis distance {axis_distance:.3f} m, observed target {target_axis_distance_m:.3f} m"
    )


def sensor_tip_observation_hint(
    depth_image, camera_info, reference_probe_pose, camera_pose_odom, expected_distance_m=None,
):
    """Locate a supported camera patch to inspect an unresolved tip target.

    The neighboring plane supplies only an observation direction. Its
    extrapolated contact is never returned as a distance measurement.
    """
    estimate = estimate_live_surface_normal(depth_image, camera_info)
    target_plane, _, axis_distance = _probe_target_geometry(
        estimate.surface_plane_camera, estimate.projected_point,
        reference_probe_pose, camera_pose_odom, expected_distance_m,
    )
    cloud = create_organized_depth_point_cloud(depth_image, camera_info)
    pixel = _observation_destination(
        depth_image, cloud, camera_info, estimate, reference_probe_pose,
        camera_pose_odom, axis_distance, expected_distance_m,
    )
    return target_plane, pixel


def _observation_destination(
    depth_image, cloud, camera_info, estimate, reference_probe_pose, camera_pose,
    axis_distance, expected_distance_m,
):
    """Find a nearby observed patch with room for camera positioning error.

    The initial plane locates candidate patches throughout the image. Each
    destination still needs its own local fit, enclosed depth support and
    bounded connection to the probe target. This guides one observation;
    fresh depth at the actual target is required after moving.
    """
    intrinsics = camera_intrinsics(camera_info, (cloud.width, cloud.height), "Depth CameraInfo")
    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose)
    target = origin + axis_distance * direction
    rows, columns = np.nonzero(cloud.valid_mask)
    points = cloud.points_camera[rows, columns]
    plane = estimate.surface_plane_camera
    residual = np.einsum("ij,j->i", points - plane.point_array(), plane.normal_array())
    margin = SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX + TARGET_SUPPORT_RADIUS_PX + 1
    candidates = np.flatnonzero(
        (columns >= margin) & (columns < cloud.width-margin)
        & (rows >= margin) & (rows < cloud.height-margin)
        & (np.abs(residual) <= DEFAULT_DEPTH_PLANE_TOLERANCE_M)
        & ~depth_below_minimum(points[:, 2], MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M)
        & (np.linalg.norm(points - target, axis=1)
           <= MAXIMUM_OBSERVATION_PLANE_EXTENSION_M)
    )
    # Require usable local target footprints throughout the positioning
    # margin. A single enlarged hull could otherwise bridge a depth hole.
    support = _observation_support_mask(cloud, plane)
    margin_pixels = np.ceil(
        max(intrinsics[:2])*OBSERVATION_TARGET_MARGIN_M/points[candidates, 2],
    ).astype(int)
    safe = np.zeros(len(candidates), dtype=bool)
    enclosed_by_radius = {}
    for radius in np.unique(margin_pixels):
        enclosed = minimum_filter(support, size=2*int(radius)+1, mode="constant", cval=0)
        enclosed_by_radius[radius] = enclosed
        selected = margin_pixels == radius
        safe[selected] = enclosed[rows[candidates[selected]], columns[candidates[selected]]]
    candidates = candidates[safe]
    pixels = np.column_stack((columns[candidates], rows[candidates]))
    translations = observation_translations_camera(
        target, direction, intrinsics, pixels,
    )
    # The target and current candidate need not have the same optical depth
    # on a tilted wall. Preserve the physical margin at both depths after
    # solving the calibrated translation, before accepting a destination.
    target_depth = target[2] - translations[:, 2]
    margin_depth = np.minimum(points[candidates, 2], target_depth)
    margin_pixels = np.ceil(max(intrinsics[:2])*OBSERVATION_TARGET_MARGIN_M/margin_depth).astype(int)
    safe = ~depth_below_minimum(target_depth, MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M)
    for radius in np.unique(margin_pixels[safe]):
        if radius not in enclosed_by_radius:
            enclosed_by_radius[radius] = minimum_filter(support, size=2*int(radius)+1, mode="constant", cval=0)
        selected = margin_pixels == radius
        safe[selected] &= enclosed_by_radius[radius][rows[candidates[selected]], columns[candidates[selected]]]
    candidates, pixels, translations, margin_depth = (
        values[safe] for values in (candidates, pixels, translations, margin_depth)
    )
    excluded = np.zeros(len(candidates), dtype=bool)
    fits = 0
    for index in np.argsort(np.einsum("ij,ij->i", translations, translations)):
        if excluded[index]:
            continue
        pixel = ImagePoint(u=int(pixels[index, 0]), v=int(pixels[index, 1]))
        support_margin = max(intrinsics[:2])*OBSERVATION_TARGET_MARGIN_M/margin_depth[index]
        support_radius = TARGET_SUPPORT_RADIUS_PX + math.ceil(support_margin)
        if fits >= MAXIMUM_SEED_FITS:
            break
        fits += 1
        try:
            projected = project_reference_pixel(
                pixel, depth_image, camera_info, search_radius_px=0,
                point_cloud=cloud, rgb_size=(cloud.width, cloud.height),
            )
            local = estimate_surface_normal(
                projected, depth_image, camera_info,
                neighborhood_radius_px=SURFACE_ORIENTATION_WINDOW_RADIUS_PX,
                maximum_neighborhood_radius_px=SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
                minimum_sample_count=SURFACE_ORIENTATION_MINIMUM_SAMPLE_COUNT,
                maximum_plane_rmse_m=DEFAULT_DEPTH_PLANE_TOLERANCE_M,
                minimum_tangent_spread_m=0.003, point_cloud=cloud,
            )
            local_plane = local.surface_plane_camera
            # A boundary patch can pass an RMSE check while fitting a very
            # different slope. It must corroborate the same wall before
            # its image location can guide an observation of this target.
            alignment = float(np.dot(plane.normal_array(), local_plane.normal_array()))
            target_offset = float(np.dot(target-local_plane.point_array(), local_plane.normal_array()))
            if (alignment < math.cos(math.radians(MAXIMUM_OBSERVATION_PLANE_NORMAL_DIFFERENCE_DEG))
                    or abs(target_offset) > DEFAULT_DEPTH_PLANE_TOLERANCE_M):
                raise ValueError("Observation patch does not agree with the target surface")
            _probe_target_geometry(
                local.surface_plane_camera, projected, reference_probe_pose,
                camera_pose, expected_distance_m,
            )
            _target_patch(
                cloud, pixel, camera_plane=local.surface_plane_camera,
                radius_px=support_radius, minimum_support_margin_px=support_margin,
            )
        except ValueError:
            radius = SURFACE_ORIENTATION_WINDOW_RADIUS_PX
            excluded |= ((pixels[:, 0]-pixel.u)**2 + (pixels[:, 1]-pixel.v)**2 <= radius**2)
            continue
        # Candidate validation must not silently redefine the target used
        # to rank translations and check its positioning margin.
        return pixel
    raise ValueError("No nearby observation patch has reliable local depth and positioning margin")


def _observation_support_mask(cloud, plane):
    """Conservatively find enclosed, foreground-free seven-pixel patches."""
    rows, columns = np.nonzero(cloud.valid_mask)
    points = cloud.points_camera[rows, columns]
    rays = points/points[:, 2, None]
    denominator = np.einsum("ij,j->i", rays, plane.normal_array())
    predicted = np.divide(
        np.dot(plane.point_array(), plane.normal_array()), denominator,
        out=np.full(len(points), np.nan), where=np.abs(denominator) > 1e-12,
    )
    residual = points[:, 2] - predicted
    consistent = np.zeros_like(cloud.valid_mask, dtype=np.uint8)
    foreground = np.zeros_like(consistent)
    consistent[rows, columns] = np.abs(residual) <= DEFAULT_DEPTH_PLANE_TOLERANCE_M
    foreground[rows, columns] = residual < -DEFAULT_DEPTH_PLANE_TOLERANCE_M
    radius = TARGET_SUPPORT_RADIUS_PX
    kernel = np.ones((2*radius+1, 2*radius+1), dtype=np.uint8)
    valid_count = correlate(cloud.valid_mask.astype(np.uint8), kernel, mode="constant", cval=0)
    consistent_count = correlate(consistent, kernel, mode="constant", cval=0)
    supported = (
        (consistent_count >= MINIMUM_TARGET_SUPPORT_SAMPLES)
        & (consistent_count >= MINIMUM_TARGET_SUPPORT_RATIO*valid_count)
        & (correlate(foreground, kernel, mode="constant", cval=0) == 0)
    )
    # One real return in each open quadrant guarantees two-dimensional
    # enclosure, including when registration leaves regularly empty pixels.
    for vertical in (slice(0, radius), slice(radius+1, None)):
        for horizontal in (slice(0, radius), slice(radius+1, None)):
            quadrant = np.zeros_like(kernel)
            quadrant[vertical, horizontal] = 1
            supported &= correlate(consistent, quadrant, mode="constant", cval=0) > 0
    return supported


def _probe_target_geometry(
    camera_plane, projected, reference_probe_pose, camera_pose_odom, expected_distance_m,
):
    """Share target-range and bounded plane-extension checks with view hints."""
    if expected_distance_m is not None and (
        not math.isfinite(expected_distance_m) or expected_distance_m <= 0.0
    ):
        raise ValueError("Expected sensor-tip distance must be positive and finite")
    plane = transform_surface_plane(
        camera_plane, relative_pose(reference_probe_pose, camera_pose_odom), "probe",
    ).oriented_toward(Vector3Data.zero())
    distance = plane.signed_distance(Vector3Data.zero())
    if not math.isfinite(distance) or plane.normal.x >= -1e-6:
        error_type = _NonForwardProbePlane if math.isfinite(distance) else ValueError
        raise error_type(
            "Measured target plane does not intersect forward probe +X: "
            f"normal_x={plane.normal.x:+.4f}, distance={distance:.3f} m, "
            f"camera depth={projected.depth_m:.3f} m at "
            f"pixel ({projected.sampled_pixel.u}, {projected.sampled_pixel.v})"
        )
    axis_distance = -distance / plane.normal.x
    if not 0.003 <= axis_distance <= 0.50:
        raise ValueError(
            f"Measured sensor-tip target is outside the 0.003–0.500 m range: {axis_distance:.3f} m"
        )
    if (expected_distance_m is not None
            and abs(axis_distance - expected_distance_m) > MAXIMUM_TARGET_DISTANCE_MISMATCH_M):
        raise ValueError(
            "Measured surface does not match the intended sensor-tip target: "
            f"axis distance {axis_distance:.3f} m, expected {expected_distance_m:.3f} m; "
            f"maximum difference {MAXIMUM_TARGET_DISTANCE_MISMATCH_M:.3f} m"
        )
    extension = float(np.linalg.norm(plane.point_array() - [axis_distance, 0.0, 0.0]))
    if extension > MAXIMUM_OBSERVATION_PLANE_EXTENSION_M:
        raise ValueError(
            "Visible surface is too far from probe +X to locate its target: "
            f"{extension:.3f} m > {MAXIMUM_OBSERVATION_PLANE_EXTENSION_M:.3f} m"
        )
    return plane, distance, axis_distance


def _next_axis_depth_seed(
    cloud, reference_probe_pose, camera_pose, intrinsics,
    center, depth_m, excluded_seeds, ray_pixels,
):
    """Exclude one rejected local seed band, retaining all ray observations."""
    radius = SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX
    patch = np.s_[max(0, center.v-radius):center.v+radius+1,
                  max(0, center.u-radius):center.u+radius+1]
    excluded_seeds[patch] |= (
        np.abs(cloud.depth_m[patch] - depth_m) <= DEFAULT_DEPTH_PLANE_TOLERANCE_M
    ) & ~ray_pixels[patch]
    return _nearest_axis_depth_pixel(
        cloud, reference_probe_pose, camera_pose, intrinsics,
        excluded_seeds=excluded_seeds,
    )


def _nearest_axis_depth_pixel(
    cloud, reference_probe_pose, camera_pose_odom, intrinsics, *, excluded_seeds=None,
):
    """Locate measured support nearest the forward ray, including near edges.

    Near returns away from the ray can be gripper pixels. Exclude them only
    when measured depth demonstrates that the ray is clear at their depth.
    Keep the original cloud for target occlusion and local support checks.
    """
    mask = cloud.valid_mask if excluded_seeds is None else cloud.valid_mask & ~excluded_seeds
    rows, columns = np.nonzero(mask)
    points = cloud.points_camera[rows, columns]
    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose_odom)
    offsets = points - origin
    # Avoid launching a BLAS thread pool for an Nx3 vector reduction while
    # the ROS executor must keep receiving fresh depth and transform data.
    axial = np.einsum("ij,j->i", offsets, direction)
    radial_squared = np.maximum(0.0, np.einsum("ij,ij->i", offsets, offsets) - axial ** 2)
    candidates = np.flatnonzero(
        (axial >= 0.003) & (axial <= 0.50)
        & (radial_squared <= MAXIMUM_OBSERVATION_PLANE_EXTENSION_M ** 2)
    )
    candidates = _exclude_off_axis_near_returns(
        cloud, rows, columns, points, origin, direction, candidates, intrinsics,
    )
    if not len(candidates):
        raise ValueError("No measured depth near forward probe +X in the 0.003–0.500 m range")
    axial_candidates = candidates[radial_squared[candidates] <= PROBE_AXIS_SEARCH_RADIUS_M ** 2]
    if len(axial_candidates):
        # Prefer the first visible surface along the ray's narrow corridor.
        # A farther wall can have a pixel closer to the mathematical ray due
        # to parallax/raster spacing; it must not hide nearer axial evidence.
        front = float(np.min(axial[axial_candidates]))
        candidates = axial_candidates[
            axial[axial_candidates] <= front + DEFAULT_DEPTH_PLANE_TOLERANCE_M
        ]
    index = candidates[np.argmin(radial_squared[candidates])]
    return ImagePoint(u=int(columns[index]), v=int(rows[index]))


def _probe_ray_pixel_mask(cloud, reference_probe_pose, camera_pose, intrinsics):
    """Identify returns in the probe footprint at their own optical depth."""
    origin, direction = probe_axis_in_camera(reference_probe_pose, camera_pose)
    if direction[2] <= 1e-6:
        return cloud.valid_mask.copy()
    rows, columns = np.nonzero(cloud.valid_mask)
    depth = cloud.depth_m[rows, columns]
    distance = (depth-origin[2])/direction[2]
    points = origin + distance[:, None]*direction
    fx, fy, cx, cy = intrinsics
    u = np.rint(fx*points[:, 0]/depth + cx)
    v = np.rint(fy*points[:, 1]/depth + cy)
    mask = np.zeros_like(cloud.valid_mask)
    mask[rows, columns] = (
        (distance >= 0.003) & (distance <= 0.50)
        & (np.abs(columns-u) <= TARGET_SUPPORT_RADIUS_PX)
        & (np.abs(rows-v) <= TARGET_SUPPORT_RADIUS_PX)
    )
    return mask


def _exclude_off_axis_near_returns(
    cloud, rows, columns, points, origin, direction, candidates, intrinsics,
):
    """Require visible free space at a near return's own probe-axis depth.

    Camera/probe parallax means a nearer obstruction can project somewhere
    other than the final wall target. An unknown or occluded near footprint
    must therefore remain blocking even if the farther wall is visible.
    """
    near = candidates[depth_below_minimum(
        points[candidates, 2], MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M,
    )]
    if not len(near) or direction[2] <= 1e-6:
        return candidates
    depth = points[near, 2]
    axis_distance = (depth - origin[2]) / direction[2]
    axis_points = origin + axis_distance[:, None] * direction
    fx, fy, cx, cy = intrinsics
    u = fx * axis_points[:, 0] / depth + cx
    v = fy * axis_points[:, 1] / depth + cy
    radius = TARGET_SUPPORT_RADIUS_PX
    eligible = (
        np.isfinite(u) & np.isfinite(v)
        & (axis_distance >= 0.003) & (axis_distance <= 0.50)
        & (u >= radius) & (u < cloud.width - radius - 1)
        & (v >= radius) & (v < cloud.height - radius - 1)
    )
    near, depth, u, v = near[eligible], depth[eligible], u[eligible], v[eligible]
    center_u, center_v = np.rint(u).astype(int), np.rint(v).astype(int)
    off_patch = ((np.abs(columns[near] - center_u) > radius)
                 | (np.abs(rows[near] - center_v) > radius))
    near, depth, u, v = near[off_patch], depth[off_patch], u[off_patch], v[off_patch]
    center_u, center_v = center_u[off_patch], center_v[off_patch]
    keep = np.ones(len(points), dtype=bool)
    # Many near pixels share one projected footprint. Check its measured
    # support once per image location instead of fitting every near pixel.
    keys = center_v * cloud.width + center_u
    for key in np.unique(keys):
        group = np.flatnonzero(keys == key)
        first, last = group[np.argmin(depth[group])], group[np.argmax(depth[group])]
        center = ImagePoint(u=int(center_u[first]), v=int(center_v[first]))
        patch = np.s_[center.v-radius:center.v+radius+1, center.u-radius:center.u+radius+1]
        observed = cloud.depth_m[patch][cloud.valid_mask[patch]]
        if (not len(observed)
                or depth_below_minimum(np.min(observed), MINIMUM_SURFACE_ORIENTATION_CAMERA_DISTANCE_M)
                or np.min(observed) <= depth[last] + DEFAULT_DEPTH_PLANE_TOLERANCE_M):
            continue
        try:
            # The projected probe ray is a line. Enclosing both endpoints
            # encloses every candidate projection within this footprint.
            _target_patch(cloud, center, projected_pixel=(u[first], v[first]))
            if last != first:
                _target_patch(cloud, center, projected_pixel=(u[last], v[last]))
        except ValueError:
            continue
        keep[near[group]] = False
    return candidates[keep[candidates]]


def depth_frame_id(depth_image, camera_info):
    image_frame = depth_image.header.frame_id.strip()
    info_frame = camera_info.header.frame_id.strip()
    if image_frame and info_frame and image_frame != info_frame:
        raise ValueError("Live depth image and CameraInfo frames differ")
    frame_id = image_frame or info_frame
    if not frame_id:
        raise ValueError("Live depth frame is empty")
    return frame_id


def _target_patch(
    cloud, center, *, projected_pixel=None, camera_plane=None, minimum_support_margin_px=0.0,
    radius_px=TARGET_SUPPORT_RADIUS_PX,
):
    """Require measured samples around a target, not a dense registered raster.

    Registration can leave regular empty pixels between valid depth returns.
    Four non-collinear returns enclosing the target protect local support;
    the larger fit still requires its own minimum sample count and spread.
    A plane must agree with the measured returns and cannot hide foreground.
    """
    radius = radius_px
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
    if np.any(hull.equations[:, -1] > -minimum_support_margin_px + 1e-9):
        raise ValueError(
            "Measured depth does not surround the intended sensor-tip target: "
            f"target is outside measured support; {detail}"
        )
    return ratio
