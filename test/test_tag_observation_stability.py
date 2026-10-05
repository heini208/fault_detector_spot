"""Validate post-boundary tag observation stability."""

import math

import pytest
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler,
)
from fault_detector_spot.sensing.observations.tag_observation_stability import (
    StableTagObservationTracker,
    TagObservationStabilityConfig,
)


def config():
    return TagObservationStabilityConfig(
        required_samples=3,
        maximum_position_span_m=0.02,
        maximum_yaw_span_rad=math.radians(2.0),
        maximum_sample_span_sec=0.5,
    )


def observation(
    stamp_sec,
    x=1.0,
    y=0.0,
    yaw=0.0,
    frame_id="body",
):
    value = TagElement()
    value.id = 7
    value.pose.header.frame_id = frame_id
    whole = int(stamp_sec)
    value.pose.header.stamp.sec = whole
    value.pose.header.stamp.nanosec = round(
        (float(stamp_sec) - whole) * 1e9
    )
    value.pose.pose.position.x = x
    value.pose.pose.position.y = y
    quaternion = quaternion_from_euler("z", yaw)
    value.pose.pose.orientation.x = quaternion.x
    value.pose.pose.orientation.y = quaternion.y
    value.pose.pose.orientation.z = quaternion.z
    value.pose.pose.orientation.w = quaternion.w
    return value


def test_requires_multiple_unique_post_boundary_samples():
    tracker = StableTagObservationTracker(config())

    assert tracker.update(observation(10.1), 10.0) is None
    assert tracker.update(observation(10.2), 10.0) is None
    accepted = tracker.update(observation(10.3), 10.0)

    assert accepted is not None
    assert accepted.pose.header.stamp.sec == 10
    assert accepted.pose.header.stamp.nanosec == 300_000_000


def test_default_yaw_span_accepts_reported_variation_but_rejects_larger_jump():
    tracker = StableTagObservationTracker(TagObservationStabilityConfig())
    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(11.1, yaw=math.radians(1)), 10.0)
    assert tracker.update(
        observation(12.1, yaw=math.radians(2.47)), 10.0,
    ) is not None
    assert tracker.update(
        observation(13.1, yaw=math.radians(5)), 10.0,
    ) is None


def test_republished_same_camera_observation_does_not_count_again():
    tracker = StableTagObservationTracker(config())
    sample = observation(10.1)

    assert tracker.update(sample, 10.0) is None
    assert tracker.update(sample, 10.0) is None
    assert tracker.update(sample, 10.0) is None
    assert tracker.sample_count == 1


def test_boundary_and_pre_boundary_samples_are_rejected():
    tracker = StableTagObservationTracker(config())

    assert tracker.update(observation(9.9), 10.0) is None
    assert tracker.update(observation(10.0), 10.0) is None
    assert tracker.sample_count == 0


def test_unstable_position_restarts_window_from_latest_sample():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1, x=1.0), 10.0)
    tracker.update(observation(10.2, x=1.005), 10.0)
    assert tracker.update(observation(10.3, x=1.1), 10.0) is None
    assert tracker.sample_count == 1

    assert tracker.update(observation(10.4, x=1.101), 10.0) is None
    assert (
        tracker.update(observation(10.5, x=1.102), 10.0)
        is not None
    )


def test_rejection_detail_survives_repeated_cached_observation():
    tracker = StableTagObservationTracker(config())
    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(10.2), 10.0)
    tracker.update(observation(10.3, x=1.1), 10.0)
    assert "Pose variation in body" in tracker.detail
    assert "0.1000 m" in tracker.detail
    detail = tracker.detail
    tracker.update(observation(10.3, x=1.1), 10.0)
    assert tracker.detail == detail


def test_unstable_yaw_restarts_window():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1, yaw=0.0), 10.0)
    tracker.update(
        observation(10.2, yaw=math.radians(0.5)),
        10.0,
    )
    assert (
        tracker.update(
            observation(10.3, yaw=math.radians(5.0)),
            10.0,
        )
        is None
    )
    assert tracker.sample_count == 1


def test_samples_outside_time_window_do_not_form_stable_result():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(10.2), 10.0)
    assert tracker.update(observation(10.71), 10.0) is None
    assert tracker.sample_count == 1


def test_new_settle_boundary_discards_previous_samples():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(10.2), 10.0)
    assert tracker.sample_count == 2

    assert tracker.update(observation(10.7), 10.6) is None
    assert tracker.sample_count == 1
    assert tracker.update(observation(10.8), 10.6) is None
    assert (
        tracker.update(observation(10.9), 10.6)
        is not None
    )


def test_tag_identity_change_resets_stability_window():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(10.2), 10.0)
    changed = observation(10.3)
    changed.id = 8

    assert tracker.update(changed, 10.0) is None
    assert tracker.sample_count == 1


def test_frame_change_resets_stability_window():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.1), 10.0)
    tracker.update(observation(10.2), 10.0)
    assert (
        tracker.update(
            observation(10.3, frame_id="other"),
            10.0,
        )
        is None
    )
    assert tracker.sample_count == 1


def test_timestamp_rewind_restarts_window():
    tracker = StableTagObservationTracker(config())

    tracker.update(observation(10.2), 10.0)
    assert tracker.update(observation(10.1), 10.0) is None
    assert tracker.sample_count == 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"required_samples": 1},
        {"required_samples": True},
        {"maximum_position_span_m": 0.0},
        {"maximum_yaw_span_rad": float("nan")},
        {"maximum_sample_span_sec": float("inf")},
    ],
)
def test_invalid_stability_config_is_rejected(kwargs):
    values = {
        "required_samples": 3,
        "maximum_position_span_m": 0.02,
        "maximum_yaw_span_rad": math.radians(2.0),
        "maximum_sample_span_sec": 0.5,
    }
    values.update(kwargs)

    with pytest.raises(ValueError):
        TagObservationStabilityConfig(**values)
