"""Tests for fixed-frame tag anchors and camera freshness."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup import (
    multi_reference_view_capture as capture_module,
)
from fault_detector_spot.inspection.data.models import PoseData, ReferenceView


class FakeSynchronizer:
    def __init__(self, inputs):
        self.inputs = inputs

    def best_snapshot(self, minimum_input_sequence):
        return self.inputs

    def collection_diagnostics(self, minimum_sequence):
        return "valid"


class FakeRepository:
    def save_reference_views(
        self,
        object_id,
        routine_id,
        captures,
        **kwargs,
    ):
        return captures


def make_inputs(rgb_sec=15, tag_sec=14):
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
    return (rgb, depth, info, info), tag


def make_request(inputs):
    return capture_module.CameraCaptureRequest(
        slot_index=0,
        camera_id="hand",
        input_synchronizer=FakeSynchronizer(inputs),
        minimum_input_sequence=1,
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


def test_tag_at_freshness_limit_is_accepted(monkeypatch):
    patch_pose_resolver(monkeypatch)
    inputs, tag = make_inputs(tag_sec=14)

    result = capture_module.capture_reference_views(
        FakeRepository(),
        [make_request(inputs)],
        object(),
        "object",
        "routine",
        SimpleNamespace(nanoseconds=15_000_000_000),
        2,
        [tag],
        maximum_input_age_sec=5.0,
    )

    assert len(result) == 1


def test_old_fixed_frame_tag_anchor_is_accepted(monkeypatch):
    patch_pose_resolver(monkeypatch)
    inputs, tag = make_inputs(tag_sec=13)
    tag.pose.header.frame_id = "odom"

    result = capture_module.capture_reference_views(
        FakeRepository(),
        [make_request(inputs)],
        object(),
        "object",
        "routine",
        SimpleNamespace(nanoseconds=15_100_000_000),
        2,
        [tag],
        maximum_input_age_sec=5.0,
    )

    assert result[0].reference_tag is tag


@pytest.mark.parametrize(
    "input_index,label",
    [
        (0, "hand RGB image is stale"),
        (1, "hand depth image is stale"),
    ],
)
def test_stale_camera_image_is_rejected(
    monkeypatch,
    input_index,
    label,
):
    patch_pose_resolver(monkeypatch)
    inputs, tag = make_inputs()
    inputs = list(inputs)
    inputs[input_index].header.stamp.sec = 13

    with pytest.raises(
        capture_module.ReferenceViewCaptureNotReady,
        match=label,
    ):
        capture_module.capture_reference_views(
            FakeRepository(),
            [make_request(tuple(inputs))],
            object(),
            "object",
            "routine",
            SimpleNamespace(nanoseconds=15_000_000_000),
            2,
            [tag],
            maximum_input_age_sec=1.5,
        )


def test_future_camera_image_is_rejected(monkeypatch):
    patch_pose_resolver(monkeypatch)
    inputs, tag = make_inputs(rgb_sec=16, tag_sec=15)

    with pytest.raises(ValueError, match="timestamp is in the future"):
        capture_module.capture_reference_views(
            FakeRepository(),
            [make_request(inputs)],
            object(),
            "object",
            "routine",
            SimpleNamespace(nanoseconds=15_000_000_000),
            2,
            [tag],
            maximum_input_age_sec=1.5,
        )


def test_closest_shared_tag_is_selected_after_camera_pairing(
    monkeypatch,
):
    patch_pose_resolver(monkeypatch)
    inputs, older_tag = make_inputs(rgb_sec=15, tag_sec=14)
    _, closer_tag = make_inputs(rgb_sec=15, tag_sec=15)

    result = capture_module.capture_reference_views(
        FakeRepository(),
        [make_request(inputs)],
        object(),
        "object",
        "routine",
        SimpleNamespace(nanoseconds=15_200_000_000),
        2,
        [older_tag, closer_tag],
        maximum_input_age_sec=1.5,
    )

    assert result[0].reference_tag is closer_tag


def test_latest_available_anchor_is_used_without_image_skew_gate(
    monkeypatch,
):
    patch_pose_resolver(monkeypatch)
    inputs, tag = make_inputs(rgb_sec=15, tag_sec=14)

    result = capture_module.capture_reference_views(
        FakeRepository(),
        [make_request(inputs)],
        object(),
        "object",
        "routine",
        SimpleNamespace(nanoseconds=15_000_000_000),
        2,
        [tag],
        maximum_input_age_sec=1.5,
    )

    assert result[0].reference_tag is tag
