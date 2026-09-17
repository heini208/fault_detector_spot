"""Tests for reference-capture TF readiness semantics."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
)


class FakeSynchronizer:
    def __init__(self, frame_id):
        self.latest_rgb_frame_id = frame_id


class FakeTfBuffer:
    def __init__(self):
        self.lookups = []

    def transform(self, *args, **kwargs):
        raise AssertionError(
            "Readiness must not transform the newest tag at its timestamp"
        )

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time,
        timeout=None,
    ):
        self.lookups.append(
            (
                target_frame,
                source_frame,
                time.nanoseconds,
                timeout.nanoseconds,
            )
        )
        return SimpleNamespace()


def make_coordinator():
    coordinator = object.__new__(ProbeReferenceCaptureCoordinator)
    coordinator.fixed_frame = "odom"
    coordinator.transform_timeout_sec = 0.05
    coordinator.tf_buffer = FakeTfBuffer()
    return coordinator


def make_tag(frame_id="body"):
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = frame_id
    tag.pose.header.stamp.sec = 123
    tag.pose.header.stamp.nanosec = 456_000_000
    tag.pose.pose.orientation.w = 1.0
    return tag


def test_readiness_checks_latest_tf_connectivity_only():
    coordinator = make_coordinator()
    synchronizers = {
        "hand": FakeSynchronizer("hand_color_image_sensor"),
        "back": FakeSynchronizer("back_fisheye_image_sensor"),
    }

    coordinator._require_tf_ready(
        synchronizers,
        ((0, "hand"), (1, "back")),
        make_tag(),
    )

    assert [
        (target, source, time_nanoseconds)
        for target, source, time_nanoseconds, _timeout in (
            coordinator.tf_buffer.lookups
        )
    ] == [
        ("odom", "body", 0),
        ("odom", "hand_color_image_sensor", 0),
        ("odom", "back_fisheye_image_sensor", 0),
    ]


def test_readiness_rejects_missing_tag_frame():
    coordinator = make_coordinator()

    with pytest.raises(
        Exception,
        match="Reference tag frame ID is not available",
    ):
        coordinator._require_tf_ready(
            {"hand": FakeSynchronizer("hand_color_image_sensor")},
            ((0, "hand"),),
            make_tag(""),
        )


def test_readiness_rejects_missing_camera_frame():
    coordinator = make_coordinator()

    with pytest.raises(
        Exception,
        match="RGB frame ID is not available",
    ):
        coordinator._require_tf_ready(
            {"hand": FakeSynchronizer("")},
            ((0, "hand"),),
            make_tag(),
        )