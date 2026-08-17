"""Live probe-tip surface-distance measurement and bounded correction."""

import math
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.open3d_depth import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.surface_plane import (
    SurfacePlane,
    fit_surface_plane,
)
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
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
    maximum_plane_rmse_m: float = 0.020,
    minimum_plane_inlier_ratio: float = 0.60,
    minimum_tangent_spread_m: float = 0.0005,
    ransac_iterations: int = 100,
) -> SurfaceDistanceSample:
    """Fit the supported surface intersecting probe local positive X."""
    _validate_parameters(
        axis_radius_m,
        minimum_distance_m,
        maximum_distance_m,
        minimum_samples,
        minimum_valid_pixel_ratio,
        maximum_plane_rmse_m,
        minimum_plane_inlier_ratio,
        minimum_tangent_spread_m,
        ransac_iterations,
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
    probe_points = _transform_points(
        camera_points,
        probe_to_camera_pose,
    )

    axial = probe_points[:, 0]
    radial_squared = probe_points[:, 1] ** 2 + probe_points[:, 2] ** 2
    roi_mask = (
        (axial >= minimum_distance_m)
        & (axial <= maximum_distance_m)
        & (radial_squared <= axis_radius_m ** 2)
    )
    candidate_count = int(np.count_nonzero(roi_mask))
    if candidate_count < minimum_samples:
        raise ValueError(
            "Live depth has insufficient support along probe local +X"
        )

    candidate_points = probe_points[roi_mask]
    plane = fit_surface_plane(
        candidate_points,
        "probe",
        maximum_plane_rmse_m,
        max(minimum_samples, 3),
        minimum_plane_inlier_ratio,
        ransac_iterations,
        minimum_tangent_spread_m,
    ).oriented_toward(Vector3Data(x=0.0, y=0.0, z=0.0))

    normal_x = float(plane.normal.x)
    if normal_x >= -1e-6:
        raise ValueError(
            "Fitted surface does not face the probe along local +X"
        )
    distance_m = _positive_x_plane_intersection(plane)
    if not minimum_distance_m <= distance_m <= maximum_distance_m:
        raise ValueError(
            "Fitted surface intersection is outside the probe distance range"
        )

    valid_pixel_ratio = float(plane.inlier_count / candidate_count)
    if valid_pixel_ratio < minimum_valid_pixel_ratio:
        raise ValueError(
            "Live depth fitted-surface support ratio is too low"
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
    stamp = depth_image.header.stamp
    return SurfaceDistanceSample(
        distance_m=distance_m,
        stamp_seconds=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        frame_id=frame_id,
        sample_count=plane.inlier_count,
        valid_pixel_ratio=valid_pixel_ratio,
        spread_m=plane.rmse_m,
        source_region=source_region,
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
    sample_span = (
        selected[-1].stamp_seconds - selected[0].stamp_seconds
    )
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
    peak_to_peak = float(np.max(distances) - np.min(distances))
    if peak_to_peak > stability_tolerance_m:
        raise ValueError("Surface-distance frames are unstable")

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
        sample for sample in selected
        if sample.surface_plane_probe is not None
    ]
    plane = fitted_samples[-1].surface_plane_probe if fitted_samples else None
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


def _positive_x_plane_intersection(plane: SurfacePlane) -> float:
    normal = plane.normal_array()
    point = plane.point_array()
    denominator = float(normal[0])
    if abs(denominator) <= 1e-6:
        raise ValueError("Surface plane is nearly parallel to probe local +X")
    distance = float(np.dot(normal, point) / denominator)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("Surface plane does not intersect probe local +X ahead")
    return distance


def _transform_points(points: np.ndarray, pose: PoseData) -> np.ndarray:
    rotation = _rotation_matrix(pose)
    translation = np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=float,
    )
    return points @ rotation.T + translation


def _rotation_matrix(pose: PoseData) -> np.ndarray:
    quaternion = pose.orientation
    x, y, z, w = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
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


def _validate_parameters(
    axis_radius_m,
    minimum_distance_m,
    maximum_distance_m,
    minimum_samples,
    minimum_valid_pixel_ratio,
    maximum_plane_rmse_m,
    minimum_plane_inlier_ratio,
    minimum_tangent_spread_m,
    ransac_iterations,
):
    for label, value in (
        ("Probe-axis radius", axis_radius_m),
        ("Minimum surface distance", minimum_distance_m),
        ("Maximum surface distance", maximum_distance_m),
        ("Maximum surface-plane RMSE", maximum_plane_rmse_m),
        ("Minimum tangent spread", minimum_tangent_spread_m),
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
    for label, value in (
        ("Minimum valid-pixel ratio", minimum_valid_pixel_ratio),
        ("Minimum plane inlier ratio", minimum_plane_inlier_ratio),
    ):
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError(f"{label} must be in the interval (0, 1]")
    if (
        isinstance(ransac_iterations, bool)
        or not isinstance(ransac_iterations, int)
        or ransac_iterations <= 0
    ):
        raise ValueError("RANSAC iteration count must be positive")
