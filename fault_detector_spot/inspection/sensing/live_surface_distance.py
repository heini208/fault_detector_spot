"""Live probe-tip surface-distance measurement and bounded correction."""

import math
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.data.models import PoseData
from fault_detector_spot.inspection.setup.reference_view_depth_projection import ImageRegion
from fault_detector_spot.inspection.utility.surface_distance_validation import (
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


def measure_probe_surface_distance(
    depth_image: Image,
    camera_info: CameraInfo,
    probe_to_camera_pose: PoseData,
    axis_radius_m: float = 0.025,
    minimum_distance_m: float = 0.003,
    maximum_distance_m: float = 0.50,
    cluster_width_m: float = 0.015,
    minimum_samples: int = 12,
    minimum_valid_pixel_ratio: float = 0.20,
    maximum_spread_m: float = 0.020,
) -> SurfaceDistanceSample:
    """Measure the nearest supported surface along probe local negative X."""
    _validate_parameters(
        axis_radius_m,
        minimum_distance_m,
        maximum_distance_m,
        cluster_width_m,
        minimum_samples,
        minimum_valid_pixel_ratio,
        maximum_spread_m,
    )
    probe_to_camera_pose.validate()
    depths = _depth_array_m(depth_image)
    fx, fy, cx, cy = _camera_intrinsics(camera_info, depth_image)
    frame_id = _frame_id(depth_image, camera_info)

    valid_depth = np.isfinite(depths) & (depths > 0.0)
    if not np.any(valid_depth):
        raise ValueError("Live registered depth contains no valid pixels")
    rows, columns = np.nonzero(valid_depth)
    z_camera = depths[rows, columns]
    x_camera = (columns.astype(float) - cx) * z_camera / fx
    y_camera = (rows.astype(float) - cy) * z_camera / fy

    rotation = _rotation_matrix(probe_to_camera_pose)
    translation = probe_to_camera_pose.position
    x_probe = (
        rotation[0, 0] * x_camera
        + rotation[0, 1] * y_camera
        + rotation[0, 2] * z_camera
        + translation.x
    )
    y_probe = (
        rotation[1, 0] * x_camera
        + rotation[1, 1] * y_camera
        + rotation[1, 2] * z_camera
        + translation.y
    )
    z_probe = (
        rotation[2, 0] * x_camera
        + rotation[2, 1] * y_camera
        + rotation[2, 2] * z_camera
        + translation.z
    )
    axial_distance = -x_probe
    cylinder = (
        (axial_distance >= minimum_distance_m)
        & (axial_distance <= maximum_distance_m)
        & (y_probe * y_probe + z_probe * z_probe <= axis_radius_m ** 2)
    )
    candidate_count = int(np.count_nonzero(cylinder))
    if candidate_count < minimum_samples:
        raise ValueError(
            "Live depth has insufficient support along the probe axis"
        )

    candidate_distances = axial_distance[cylinder]
    seed = float(np.percentile(candidate_distances, 20.0))
    cluster = np.abs(candidate_distances - seed) <= cluster_width_m
    cluster_count = int(np.count_nonzero(cluster))
    if cluster_count < minimum_samples:
        raise ValueError(
            "Live depth has no supported nearest-surface cluster"
        )
    valid_pixel_ratio = float(cluster_count / candidate_count)
    if valid_pixel_ratio < minimum_valid_pixel_ratio:
        raise ValueError(
            "Live depth nearest-surface support ratio is too low"
        )
    cluster_distances = candidate_distances[cluster]
    distance_m = float(np.median(cluster_distances))
    lower, upper = np.percentile(cluster_distances, [10.0, 90.0])
    spread_m = float(upper - lower)
    if spread_m > maximum_spread_m:
        raise ValueError(
            "Live surface-distance cluster is too dispersed"
        )

    selected_rows = rows[cylinder][cluster]
    selected_columns = columns[cylinder][cluster]
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
        stamp_seconds=(
            float(stamp.sec) + float(stamp.nanosec) * 1e-9
        ),
        frame_id=frame_id,
        sample_count=cluster_count,
        valid_pixel_ratio=valid_pixel_ratio,
        spread_m=spread_m,
        source_region=source_region,
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
            "Need at least three distinct post-settle depth frames"
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
    return SurfaceDistanceAggregate(
        distance_m=distance,
        frame_id=selected[-1].frame_id,
        sample_count=len(selected),
        sample_span_sec=sample_span,
        peak_to_peak_m=peak_to_peak,
        verified=verified,
        correction=correction,
    )


def _recent_sample_window(
    samples,
    minimum_samples,
    minimum_span_sec,
):
    """Return the smallest newest suffix spanning the required window."""
    newest_stamp = samples[-1].stamp_seconds
    for index in range(len(samples) - 2, -1, -1):
        if (
            len(samples) - index >= minimum_samples
            and newest_stamp - samples[index].stamp_seconds + 1e-9
            >= minimum_span_sec
        ):
            return samples[index:]
    return samples


def _depth_array_m(depth_image: Image) -> np.ndarray:
    if depth_image.width <= 0 or depth_image.height <= 0:
        raise ValueError("Live depth dimensions must be positive")
    encoding = depth_image.encoding.strip().lower()
    if encoding == "16uc1":
        dtype = np.dtype(">u2" if depth_image.is_bigendian else "<u2")
        scale = 0.001
    elif encoding == "32fc1":
        dtype = np.dtype(">f4" if depth_image.is_bigendian else "<f4")
        scale = 1.0
    else:
        raise ValueError(
            f"Unsupported live depth encoding: {depth_image.encoding}"
        )
    minimum_step = depth_image.width * dtype.itemsize
    if depth_image.step < minimum_step:
        raise ValueError("Live depth step is too small")
    expected_length = depth_image.step * depth_image.height
    if len(depth_image.data) < expected_length:
        raise ValueError("Live depth data is shorter than expected")
    raw = np.ndarray(
        shape=(depth_image.height, depth_image.width),
        dtype=dtype,
        buffer=bytes(depth_image.data),
        strides=(depth_image.step, dtype.itemsize),
    )
    values = raw.astype(float) * scale
    values[values <= 0.0] = np.nan
    return values


def _camera_intrinsics(camera_info, depth_image):
    if len(camera_info.k) != 9:
        raise ValueError("Live depth CameraInfo must contain nine intrinsics")
    if camera_info.width not in (0, depth_image.width):
        raise ValueError("Live depth CameraInfo width does not match")
    if camera_info.height not in (0, depth_image.height):
        raise ValueError("Live depth CameraInfo height does not match")
    values = tuple(
        float(value)
        for value in (
            camera_info.k[0],
            camera_info.k[4],
            camera_info.k[2],
            camera_info.k[5],
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Live depth CameraInfo contains non-finite values")
    if values[0] <= 0.0 or values[1] <= 0.0:
        raise ValueError("Live depth focal lengths must be positive")
    return values


def _frame_id(depth_image, camera_info):
    image_frame = depth_image.header.frame_id.strip()
    info_frame = camera_info.header.frame_id.strip()
    if image_frame and info_frame and image_frame != info_frame:
        raise ValueError("Live depth image and CameraInfo frames differ")
    frame_id = image_frame or info_frame
    if not frame_id:
        raise ValueError("Live depth frame is empty")
    return frame_id


def _rotation_matrix(pose):
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


def _validate_parameters(
    axis_radius_m,
    minimum_distance_m,
    maximum_distance_m,
    cluster_width_m,
    minimum_samples,
    minimum_valid_pixel_ratio,
    maximum_spread_m,
):
    for label, value in (
        ("Probe-axis radius", axis_radius_m),
        ("Minimum surface distance", minimum_distance_m),
        ("Maximum surface distance", maximum_distance_m),
        ("Surface cluster width", cluster_width_m),
        ("Maximum surface spread", maximum_spread_m),
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
