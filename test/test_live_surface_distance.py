"""Tests for live probe-axis distance measurement and correction."""

import math
import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.sensing.live_surface_distance import (
    SurfaceDistanceSample,
    aggregate_surface_distance_samples,
    bounded_surface_distance_correction,
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)


def planar_depth(width=21, height=21, depth_m=0.20):
    image = Image()
    image.header.frame_id = "hand_depth"
    image.header.stamp.sec = 10
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.step = width * 4
    image.data = b"".join(
        struct.pack("<f", depth_m)
        for _ in range(width * height)
    )
    info = CameraInfo()
    info.header.frame_id = "hand_depth"
    info.width = width
    info.height = height
    info.k = [
        100.0,
        0.0,
        float(width // 2),
        0.0,
        100.0,
        float(height // 2),
        0.0,
        0.0,
        1.0,
    ]
    return image, info


def probe_to_camera_pose():
    half_angle = math.radians(90.0) * 0.5
    return PoseData(
        position=Vector3Data(x=-0.10, y=0.0, z=0.0),
        orientation=QuaternionData(
            x=0.0,
            y=math.sin(half_angle),
            z=0.0,
            w=math.cos(half_angle),
        ),
    )


def opposite_probe_to_camera_pose():
    half_angle = math.radians(-90.0) * 0.5
    return PoseData(
        position=Vector3Data(x=0.10, y=0.0, z=0.0),
        orientation=QuaternionData(
            x=0.0,
            y=math.sin(half_angle),
            z=0.0,
            w=math.cos(half_angle),
        ),
    )


def distance_sample(distance_m, stamp_seconds, frame_id="hand_depth"):
    return SurfaceDistanceSample(
        distance_m=distance_m,
        stamp_seconds=stamp_seconds,
        frame_id=frame_id,
        sample_count=30,
        valid_pixel_ratio=0.8,
        spread_m=0.001,
        source_region=ImageRegion(x=0, y=0, width=3, height=3),
    )


def test_measurement_reports_positive_x_tip_to_surface_distance():
    image, info = planar_depth()

    sample = measure_probe_surface_distance(
        image,
        info,
        probe_to_camera_pose(),
    )

    assert sample.distance_m == pytest.approx(0.10, abs=1e-6)
    assert sample.frame_id == "hand_depth"
    assert sample.stamp_seconds == pytest.approx(10.0)
    assert sample.sample_count >= 12
    assert sample.spread_m == pytest.approx(0.0, abs=1e-6)


def test_measurement_rejects_surface_behind_probe_axis():
    image, info = planar_depth()

    with pytest.raises(ValueError, match="local \\+X"):
        measure_probe_surface_distance(
            image,
            info,
            opposite_probe_to_camera_pose(),
        )


@pytest.mark.parametrize(
    "measured,target,expected",
    [
        (0.05, 0.03, 0.01),
        (0.01, 0.03, -0.01),
        (0.0305, 0.03, 0.0),
    ],
)
def test_correction_is_directional_and_bounded(
    measured,
    target,
    expected,
):
    result = bounded_surface_distance_correction(
        measured,
        target,
        maximum_step_m=0.01,
    )

    assert result.inward_correction_m == pytest.approx(expected)


def test_measurement_rejects_depth_without_probe_axis_support():
    image, info = planar_depth(depth_m=1.0)

    with pytest.raises(ValueError, match="local \\+X"):
        measure_probe_surface_distance(
            image,
            info,
            probe_to_camera_pose(),
        )


def test_three_stable_frames_verify_only_when_every_frame_is_in_tolerance():
    samples = [
        distance_sample(0.029, 10.0),
        distance_sample(0.031, 10.1),
        distance_sample(0.030, 10.2),
    ]

    result = aggregate_surface_distance_samples(
        samples,
        target_distance_m=0.03,
        maximum_step_m=0.02,
    )

    assert result.verified
    assert result.sample_count == 3
    assert result.sample_span_sec == pytest.approx(0.2)
    assert result.peak_to_peak_m == pytest.approx(0.002)
    assert result.correction.inward_correction_m == 0.0


def test_aggregate_returns_one_bounded_directional_correction():
    samples = [
        distance_sample(0.079, 10.0),
        distance_sample(0.080, 10.1),
        distance_sample(0.081, 10.2),
    ]

    result = aggregate_surface_distance_samples(
        samples,
        target_distance_m=0.03,
        maximum_step_m=0.02,
    )

    assert not result.verified
    assert result.distance_m == pytest.approx(0.08)
    assert result.correction.inward_correction_m == pytest.approx(0.02)


def test_aggregate_uses_the_newest_complete_sampling_window():
    samples = [
        distance_sample(0.20, 9.0),
        distance_sample(0.079, 10.0),
        distance_sample(0.080, 10.1),
        distance_sample(0.081, 10.2),
    ]

    result = aggregate_surface_distance_samples(
        samples,
        target_distance_m=0.03,
        maximum_step_m=0.02,
    )

    assert result.sample_count == 3
    assert result.distance_m == pytest.approx(0.08)


def test_aggregate_rejects_duplicate_or_short_sampling_window():
    duplicate_samples = [
        distance_sample(0.08, 10.0),
        distance_sample(0.08, 10.0),
        distance_sample(0.08, 10.2),
    ]
    short_samples = [
        distance_sample(0.08, 10.0),
        distance_sample(0.08, 10.05),
        distance_sample(0.08, 10.10),
    ]

    with pytest.raises(ValueError, match="at least 3 distinct"):
        aggregate_surface_distance_samples(
            duplicate_samples,
            target_distance_m=0.03,
            maximum_step_m=0.02,
        )
    with pytest.raises(ValueError, match="sampling window"):
        aggregate_surface_distance_samples(
            short_samples,
            target_distance_m=0.03,
            maximum_step_m=0.02,
        )


def test_aggregate_rejects_unstable_or_directionally_mixed_frames():
    unstable = [
        distance_sample(0.028, 10.0),
        distance_sample(0.032, 10.1),
        distance_sample(0.038, 10.2),
    ]
    directionally_mixed = [
        distance_sample(0.024, 10.0),
        distance_sample(0.030, 10.1),
        distance_sample(0.036, 10.2),
    ]

    with pytest.raises(ValueError, match="unstable"):
        aggregate_surface_distance_samples(
            unstable,
            target_distance_m=0.03,
            maximum_step_m=0.02,
        )
    with pytest.raises(ValueError, match="direction"):
        aggregate_surface_distance_samples(
            directionally_mixed,
            target_distance_m=0.03,
            maximum_step_m=0.02,
            stability_tolerance_m=0.02,
        )
