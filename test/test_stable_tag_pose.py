"""Tests for fresh, distinct base-camera tag stabilization."""

import math

import pytest

from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.stable_tag_pose import (
    TagPoseSample,
    stabilize_tag_pose,
)


def sample(stamp, x=1.0, orientation=None, frame="body"):
    """Create one base-camera tag observation."""
    return TagPoseSample(
        stamp_seconds=stamp,
        frame_id=frame,
        pose=PoseData(
            position=Vector3Data(x=x, y=0.0, z=0.0),
            orientation=orientation or QuaternionData.identity(),
        ),
    )


def test_high_rate_samples_use_the_whole_fresh_stabilization_window():
    samples = [sample(10.0 + index * 0.03) for index in range(8)]

    stable = stabilize_tag_pose(samples, now_seconds=10.22)

    assert stable.sample_count == 5
    assert stable.sample_span_sec == pytest.approx(0.12)
    assert stable.pose.position.x == pytest.approx(1.0)


def test_duplicate_timestamps_do_not_count_as_distinct_observations():
    samples = [
        sample(10.0),
        sample(10.0, x=1.001),
        sample(10.1),
    ]

    with pytest.raises(ValueError, match="three distinct"):
        stabilize_tag_pose(samples, now_seconds=10.11)


def test_stabilization_rejects_position_jitter():
    samples = [
        sample(10.0, x=1.0),
        sample(10.1, x=1.0),
        sample(10.2, x=1.04),
    ]

    with pytest.raises(ValueError, match="position is not stable"):
        stabilize_tag_pose(samples, now_seconds=10.21)


def test_quaternion_sign_does_not_create_false_orientation_jitter():
    positive = QuaternionData.identity()
    negative = QuaternionData(x=0.0, y=0.0, z=0.0, w=-1.0)
    samples = [
        sample(10.0, orientation=positive),
        sample(10.1, orientation=negative),
        sample(10.2, orientation=positive),
    ]

    stable = stabilize_tag_pose(samples, now_seconds=10.21)

    assert math.isclose(abs(stable.pose.orientation.w), 1.0)
    assert stable.maximum_orientation_deviation_rad == pytest.approx(0.0)


def test_stabilization_rejects_stale_observations():
    samples = [sample(9.0), sample(9.1), sample(9.2)]

    with pytest.raises(ValueError, match="fresh"):
        stabilize_tag_pose(samples, now_seconds=10.0)
