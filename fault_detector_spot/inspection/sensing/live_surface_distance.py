"""Live probe-tip surface-distance measurement and bounded correction."""

import math
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.shared.geometry.rotation import (
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.geometry.surface_plane import (
    DEFAULT_DEPTH_PLANE_TOLERANCE_M,
    SurfacePlane,
    fit_surface_plane,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
)
from fault_detector_spot.inspection.sensing.live_surface_plane import (
    estimate_live_surface_normal,
)


@dataclass(frozen=True)
class SurfaceDistanceSample:
    """One quality-checked probe-tip distance sample."""

    distance_m: float
    stamp_seconds: float
    frame_id: str
    sample_count: int
    valid_pixel_ratio: float
    spread_m: float
    source_region: ImageRegion
    surface_plane_probe: SurfacePlane = None


@dataclass(frozen=True)
class SurfaceDistanceCorrection:
    """One bounded correction along the probe's local X axis."""

    measured_distance_m: float
    target_distance_m: float
    error_m: float
    inward_correction_m: float


@dataclass(frozen=True)
class SurfaceDistanceAggregate:
    """Stable multi-frame distance used for one supervised decision."""

    distance_m: float
    frame_id: str
    sample_count: int
    sample_span_sec: float
    peak_to_peak_m: float
    verified: bool
    correction: SurfaceDistanceCorrection
    surface_plane_probe: SurfacePlane = None


def measure_probe_surface_distance(
    depth_image: Image,
    camera_info: CameraInfo,
    probe_to_camera_pose: PoseData,
    axis_radius_m: float = 0.025,
    minimum_distance_m: float = 0.003,
    maximum_distance_m: float = 0.50,
    minimum_samples: int = 12,
    minimum_valid_pixel_ratio: float = 0.20,
    maximum_plane_extrapolation_m: float = 0.15,
) -> SurfaceDistanceSample:
    """Measure probe distance, using a nearby visible plane for missing axial depth."""
    _validate_measurement_parameters(
        axis_radius_m,
        minimum_distance_m,
        maximum_distance_m,
        minimum_samples,
        minimum_valid_pixel_ratio,
    )
    require_positive_finite_distance(
        maximum_plane_extrapolation_m, "Maximum surface-plane extrapolation",
    )
    probe_to_camera_pose.validate()
    cloud = create_organized_depth_point_cloud(depth_image, camera_info)
    frame_id = _frame_id(depth_image, camera_info)

    valid_rows, valid_columns = np.nonzero(cloud.valid_mask)
    if len(valid_rows) == 0:
        raise ValueError("Live registered depth contains no valid pixels")
    camera_points = np.asarray(
        cloud.points_camera[valid_rows, valid_columns],
        dtype=np.float64,
    )
    probe_points = _transform_points(camera_points, probe_to_camera_pose)

    axial = probe_points[:, 0]
    radial_squared = probe_points[:, 1] ** 2 + probe_points[:, 2] ** 2
    axis_mask = radial_squared <= axis_radius_m ** 2
    axis_count = int(np.count_nonzero(axis_mask))
    roi_mask = (
        axis_mask
        & (axial >= minimum_distance_m)
        & (axial <= maximum_distance_m)
    )
    candidate_count = int(np.count_nonzero(roi_mask))
    if axis_count == 0:
        try:
            return _nearby_plane_distance_sample(
                depth_image, camera_info, probe_to_camera_pose,
                minimum_distance_m, maximum_distance_m, minimum_samples,
                minimum_valid_pixel_ratio, maximum_plane_extrapolation_m,
            )
        except ValueError as exception:
            raise ValueError(
                "Live depth has no support along probe local +X; "
                f"nearby surface-plane estimate rejected: {exception}"
            ) from exception
    # Sparse axial depth can still reveal an obstacle or an out-of-range
    # surface. Never replace that evidence with a neighboring plane.
    if candidate_count < minimum_samples:
        raise ValueError("Live depth has insufficient support along probe local +X")

    valid_pixel_ratio = float(candidate_count / axis_count)
    if valid_pixel_ratio < minimum_valid_pixel_ratio:
        raise ValueError(
            "Live depth valid support ratio along probe local +X is too low"
        )

    candidate_distances = np.asarray(axial[roi_mask], dtype=np.float64)
    distance_m = float(np.median(candidate_distances))
    if not math.isfinite(distance_m):
        raise ValueError("Estimated probe surface distance is not finite")
    spread_m = float(
        np.median(np.abs(candidate_distances - distance_m))
    )

    selected_rows = valid_rows[roi_mask]
    selected_columns = valid_columns[roi_mask]
    source_region = ImageRegion(
        x=int(np.min(selected_columns)),
        y=int(np.min(selected_rows)),
        width=int(np.max(selected_columns) - np.min(selected_columns) + 1),
        height=int(np.max(selected_rows) - np.min(selected_rows) + 1),
    )
    source_region.validate()

    camera_plane = fit_surface_plane(
        camera_points[roi_mask], frame_id, DEFAULT_DEPTH_PLANE_TOLERANCE_M, minimum_samples,
        0.60, 100, 0.003,
        depth_camera=True,
    )
    reference_plane = _plane_in_probe_frame(camera_plane, probe_to_camera_pose)
    if reference_plane.rmse_m > DEFAULT_DEPTH_PLANE_TOLERANCE_M:
        raise ValueError("Probe surface plane residual exceeds 15 mm")
    # Use perpendicular distance consistently with frozen-plane tracking.
    distance_m = reference_plane.signed_distance(Vector3Data(0.0, 0.0, 0.0))
    stamp = depth_image.header.stamp
    return SurfaceDistanceSample(
        distance_m=distance_m,
        stamp_seconds=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        frame_id=frame_id,
        sample_count=candidate_count,
        valid_pixel_ratio=valid_pixel_ratio,
        spread_m=spread_m,
        source_region=source_region,
        surface_plane_probe=reference_plane,
    )


def _plane_in_probe_frame(camera_plane, probe_to_camera_pose):
    point_probe = _transform_points(camera_plane.point_array()[None, :], probe_to_camera_pose)[0]
    normal_probe = rotation_from_quaternion(probe_to_camera_pose.orientation).apply(
        camera_plane.normal_array(),
    )
    return SurfacePlane(
        point=Vector3Data(*point_probe), normal=Vector3Data(*normal_probe),
        frame_id="probe", inlier_count=camera_plane.inlier_count,
        sample_count=camera_plane.sample_count, inlier_ratio=camera_plane.inlier_ratio,
        rmse_m=camera_plane.rmse_m,
    ).oriented_toward(Vector3Data.zero())


def _nearby_plane_distance_sample(
    depth_image, camera_info, probe_to_camera_pose,
    minimum_distance_m, maximum_distance_m, minimum_samples,
    minimum_valid_pixel_ratio, maximum_plane_extrapolation_m,
):
    # The mount can obscure the probe-axis depth. Use the same measured patch
    # as orientation, retaining its fitted intercept rather than a raw pixel.
    estimate = estimate_live_surface_normal(depth_image, camera_info)
    camera_plane = estimate.surface_plane_camera
    if camera_plane is None:
        raise ValueError("Visible surface estimate has no fitted plane")
    camera_plane.validate()
    if (camera_plane.inlier_count < minimum_samples
            or camera_plane.inlier_ratio < minimum_valid_pixel_ratio):
        raise ValueError("Visible surface plane has insufficient fitted support")
    plane = _plane_in_probe_frame(camera_plane, probe_to_camera_pose)
    distance = plane.signed_distance(Vector3Data.zero())
    if not math.isfinite(distance) or plane.normal.x >= -1e-6:
        raise ValueError("Visible surface plane does not intersect forward probe +X")
    axis_distance = -distance / plane.normal.x
    if not minimum_distance_m <= axis_distance <= maximum_distance_m:
        raise ValueError(
            f"Visible surface intersects probe +X at {axis_distance:.3f} m; "
            f"required range is {minimum_distance_m:.3f}–{maximum_distance_m:.3f} m"
        )
    # Bound the extension along the fitted plane from its measured centroid
    # to the calibrated probe +X intersection. The default 15 cm allowance
    # accommodates the lower mounted tip while still rejecting distant patches.
    extrapolation = math.sqrt(
        (axis_distance - plane.point.x) ** 2
        + plane.point.y ** 2 + plane.point.z ** 2
    )
    if extrapolation > maximum_plane_extrapolation_m:
        raise ValueError(
            f"Visible surface patch is {extrapolation:.3f} m from probe +X; "
            f"maximum plane extrapolation is {maximum_plane_extrapolation_m:.3f} m"
        )
    center = estimate.projected_point.sampled_pixel
    radius = estimate.neighborhood_radius_px
    x0, y0 = max(0, center.u - radius), max(0, center.v - radius)
    x1 = min(int(depth_image.width), center.u + radius + 1)
    y1 = min(int(depth_image.height), center.v + radius + 1)
    return SurfaceDistanceSample(
        distance_m=distance,
        stamp_seconds=estimate.stamp_nanoseconds * 1e-9,
        frame_id=estimate.projected_point.frame_id,
        sample_count=camera_plane.inlier_count,
        valid_pixel_ratio=camera_plane.inlier_ratio,
        spread_m=camera_plane.rmse_m,
        source_region=ImageRegion(x=x0, y=y0, width=x1-x0, height=y1-y0),
        surface_plane_probe=plane,
    )


def bounded_surface_distance_correction(
    measured_distance_m: float,
    target_distance_m: float,
    maximum_step_m: float,
    tolerance_m: float = 0.005,
) -> SurfaceDistanceCorrection:
    """Return one finite inward correction, positive toward the surface."""
    for label, value in (
        ("Measured surface distance", measured_distance_m),
        ("Target surface distance", target_distance_m),
        ("Maximum correction step", maximum_step_m),
        ("Surface-distance tolerance", tolerance_m),
    ):
        require_positive_finite_distance(value, label)
    error_m = measured_distance_m - target_distance_m
    if abs(error_m) <= tolerance_m:
        inward_correction_m = 0.0
    else:
        inward_correction_m = max(
            -maximum_step_m,
            min(maximum_step_m, error_m),
        )
    return SurfaceDistanceCorrection(
        measured_distance_m=float(measured_distance_m),
        target_distance_m=float(target_distance_m),
        error_m=float(error_m),
        inward_correction_m=float(inward_correction_m),
    )


def aggregate_surface_distance_samples(
    samples,
    target_distance_m: float,
    maximum_step_m: float,
    tolerance_m: float = 0.005,
    minimum_samples: int = 3,
    minimum_span_sec: float = 0.20,
    stability_tolerance_m: float = 0.005,
) -> SurfaceDistanceAggregate:
    """Validate distinct post-settle frames and derive one correction."""
    for label, value in (
        ("Target surface distance", target_distance_m),
        ("Maximum correction step", maximum_step_m),
        ("Surface-distance tolerance", tolerance_m),
        ("Minimum sampling span", minimum_span_sec),
        ("Distance stability tolerance", stability_tolerance_m),
    ):
        require_positive_finite_distance(value, label)
    if (
        isinstance(minimum_samples, bool)
        or not isinstance(minimum_samples, int)
        or minimum_samples < 3
    ):
        raise ValueError(
            "At least three surface-distance frames are required"
        )

    distinct = {}
    for sample in samples:
        if not isinstance(sample, SurfaceDistanceSample):
            raise TypeError(
                "Surface-distance aggregate requires measured samples"
            )
        distinct[sample.stamp_seconds] = sample
    ordered = [distinct[stamp] for stamp in sorted(distinct)]
    if len(ordered) < minimum_samples:
        raise ValueError(
            f"Need at least {minimum_samples} distinct post-settle depth frames"
        )
    selected = _recent_sample_window(
        ordered,
        minimum_samples,
        minimum_span_sec,
    )
    frames = {sample.frame_id for sample in selected}
    if len(frames) != 1:
        raise ValueError("Surface-distance frames do not share one frame")
    sample_span = selected[-1].stamp_seconds - selected[0].stamp_seconds
    if sample_span + 1e-9 < minimum_span_sec:
        raise ValueError(
            "Depth frames do not span the required sampling window"
        )

    distances = np.array(
        [sample.distance_m for sample in selected],
        dtype=float,
    )
    if not np.all(np.isfinite(distances)):
        raise ValueError("Surface-distance samples must be finite")
    minimum_distance = float(np.min(distances))
    maximum_distance = float(np.max(distances))
    peak_to_peak = maximum_distance - minimum_distance
    if peak_to_peak > stability_tolerance_m:
        raise ValueError(
            "Surface-distance frames are unstable: "
            f"peak-to-peak {peak_to_peak * 1000.0:.2f} mm > "
            f"{stability_tolerance_m * 1000.0:.2f} mm limit; "
            f"distance range {minimum_distance * 1000.0:.2f}–"
            f"{maximum_distance * 1000.0:.2f} mm across "
            f"{len(selected)} selected frames spanning {sample_span:.3f} s"
        )

    errors = distances - target_distance_m
    directions = {
        1 if error > tolerance_m else -1
        for error in errors
        if abs(error) > tolerance_m
    }
    if len(directions) > 1:
        raise ValueError(
            "Surface-distance samples disagree about correction direction"
        )

    distance = float(np.median(distances))
    verified = bool(np.all(np.abs(errors) <= tolerance_m))
    correction = bounded_surface_distance_correction(
        distance,
        target_distance_m,
        maximum_step_m,
        tolerance_m=tolerance_m,
    )
    if verified and correction.inward_correction_m != 0.0:
        raise RuntimeError("Verified distance produced a correction")

    fitted_samples = [
        sample
        for sample in selected
        if sample.surface_plane_probe is not None
    ]
    representative = _representative_fitted_sample(
        fitted_samples,
        distance,
    )
    plane = (
        representative.surface_plane_probe
        if representative is not None
        else None
    )
    return SurfaceDistanceAggregate(
        distance_m=distance,
        frame_id=selected[-1].frame_id,
        sample_count=len(selected),
        sample_span_sec=sample_span,
        peak_to_peak_m=peak_to_peak,
        verified=verified,
        correction=correction,
        surface_plane_probe=plane,
    )


def _representative_fitted_sample(samples, median_distance_m):
    if not samples:
        return None
    return min(
        samples,
        key=lambda sample: (
            abs(sample.distance_m - median_distance_m),
            -sample.stamp_seconds,
        ),
    )


def _transform_points(points: np.ndarray, pose: PoseData) -> np.ndarray:
    pose.validate()
    translation = np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=float,
    )
    return rotation_from_quaternion(pose.orientation).apply(points) + translation


def _frame_id(depth_image, camera_info):
    image_frame = depth_image.header.frame_id.strip()
    info_frame = camera_info.header.frame_id.strip()
    if image_frame and info_frame and image_frame != info_frame:
        raise ValueError("Live depth image and CameraInfo frames differ")
    frame_id = image_frame or info_frame
    if not frame_id:
        raise ValueError("Live depth frame is empty")
    return frame_id


def _recent_sample_window(samples, minimum_samples, minimum_span_sec):
    newest_stamp = samples[-1].stamp_seconds
    for index in range(len(samples) - 2, -1, -1):
        if (
            len(samples) - index >= minimum_samples
            and newest_stamp - samples[index].stamp_seconds + 1e-9
            >= minimum_span_sec
        ):
            return samples[index:]
    return samples


def _validate_measurement_parameters(
    axis_radius_m,
    minimum_distance_m,
    maximum_distance_m,
    minimum_samples,
    minimum_valid_pixel_ratio,
):
    for label, value in (
        ("Probe-axis radius", axis_radius_m),
        ("Minimum surface distance", minimum_distance_m),
        ("Maximum surface distance", maximum_distance_m),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be positive and finite")
    if maximum_distance_m <= minimum_distance_m:
        raise ValueError(
            "Maximum surface distance must exceed the minimum"
        )
    if (
        isinstance(minimum_samples, bool)
        or not isinstance(minimum_samples, int)
        or minimum_samples <= 0
    ):
        raise ValueError("Minimum surface samples must be a positive integer")
    if (
        not math.isfinite(minimum_valid_pixel_ratio)
        or not 0.0 < minimum_valid_pixel_ratio <= 1.0
    ):
        raise ValueError(
            "Minimum valid-pixel ratio must be in the interval (0, 1]"
        )
