"""Tests for known-good tag freshness and RGB skew validation."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import (
    multi_reference_view_capture as capture_module,
)
from fault_detector_spot.inspection.models import PoseData, ReferenceView


class FakeSynchronizer:
    def __init__(self, inputs):
        self.inputs = inputs

    def best_snapshot(self, reference_tag_id, minimum_image_sequence):
        return self.inputs

    def collection_diagnostics(self, reference_tag_id, minimum_sequence):
        return "valid"


class FakeRepository:
    def save_reference_views(self, object_id, routine_id, captures):
        return captures


def make_inputs(rgb_sec=15, tag_sec=10):
    rgb = Image()
    rgb.header.frame_id = "camera_frame"
    rgb.header.stamp.sec = rgb_sec
    rgb.width = 1
    rgb.height = 1
    rgb.encoding = "rgb8"
    rgb.step = 3
    rgb.data = bytes([0, 0, 0])

    depth = Image()
    depth.header.frame_id = "camera_frame"
    depth.header.stamp.sec = rgb_sec
    depth.width = 1
    depth.height = 1
    depth.encoding = "16UC1"
    depth.step = 2
    depth.data = bytes([1, 0])

    info = CameraInfo()
    info.header.frame_id = "camera_frame"
    info.width = 1
    info.height = 1
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = tag_sec
    tag.pose.pose.orientation.w = 1.0
    return rgb, depth, info, tag


def make_request(inputs):
    return capture_module.CameraCaptureRequest(
        slot_index=0,
        camera_id="hand",
        input_synchronizer=FakeSynchronizer(inputs),
        minimum_image_sequence=1,
    )


def patch_pose_resolver(monkeypatch):
    reference_view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="camera_frame",
    )
    monkeypatch.setattr(
        capture_module,
        "resolve_reference_view_pose",
        lambda *args, **kwargs: reference_view,
    )


def test_tag_at_five_second_limit_is_accepted(monkeypatch):
    patch_pose_resolver(monkeypatch)

    result = capture_module.capture_reference_views(
        FakeRepository(),
        [make_request(make_inputs())],
        object(),
        "object",
        "routine",
        SimpleNamespace(nanoseconds=15_000_000_000),
        2,
        maximum_input_age_sec=5.0,
        maximum_tag_timestamp_skew_sec=5.0,
    )

    assert len(result) == 1


def test_tag_older_than_input_age_is_rejected(monkeypatch):
    patch_pose_resolver(monkeypatch)

    with pytest.raises(
        capture_module.ReferenceViewCaptureNotReady,
        match="Base-camera tag observation is stale",
    ):
        capture_module.capture_reference_views(
            FakeRepository(),
            [make_request(make_inputs())],
            object(),
            "object",
            "routine",
            SimpleNamespace(nanoseconds=15_100_000_000),
            2,
            maximum_input_age_sec=5.0,
            maximum_tag_timestamp_skew_sec=6.0,
        )
