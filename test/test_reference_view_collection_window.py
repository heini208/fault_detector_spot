"""Tests for bounded reference-view candidate collection."""

from collections import deque
from threading import Lock

import pytest
from fault_detector_msgs.msg import TagElement, TagElementArray
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import (
    reference_view_input_synchronizer as synchronizer_module,
)


ReferenceViewInputSynchronizer = (
    synchronizer_module.ReferenceViewInputSynchronizer
)


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._camera_info = CameraInfo()
    synchronizer._image_sequence = 0
    synchronizer._image_history = deque(maxlen=60)
    synchronizer._tag_history_size = 60
    synchronizer._base_tags = {}
    synchronizer._visible_tag_ids = set()
    synchronizer._maximum_timestamp_skew_nanoseconds = 50_000_000
    synchronizer._maximum_tag_timestamp_skew_nanoseconds = (
        250_000_000
    )
    synchronizer._collection_duration_nanoseconds = 1_000_000_000
    synchronizer._collection = None
    return synchronizer


def make_image(nanosec):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def make_tag(nanosec):
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_tag_array(*tags):
    message = TagElementArray()
    message.elements = list(tags)
    return message


def test_collection_waits_one_second_then_selects_best_set(
    monkeypatch,
):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer.begin_collection(7, 1)

    synchronizer._base_tags_callback(
        make_tag_array(
            make_tag(150_000_000),
            make_tag(510_000_000),
        )
    )
    synchronizer._synchronized_images_callback(
        make_image(100_000_000),
        make_image(110_000_000),
    )
    synchronizer._synchronized_images_callback(
        make_image(500_000_000),
        make_image(505_000_000),
    )

    assert synchronizer.collection_ready() is False
    with pytest.raises(RuntimeError, match="not complete"):
        synchronizer.best_snapshot(7, 1)

    clock[0] += 1_000_000_000
    assert synchronizer.collection_ready() is True
    snapshot = synchronizer.best_snapshot(7, 1)

    assert snapshot[0].header.stamp.nanosec == 500_000_000
    assert snapshot[1].header.stamp.nanosec == 505_000_000
    assert snapshot[3].pose.header.stamp.nanosec == 510_000_000


def test_collection_excludes_images_before_minimum_sequence(
    monkeypatch,
):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer._synchronized_images_callback(
        make_image(100_000_000),
        make_image(105_000_000),
    )
    synchronizer.begin_collection(7, 2)
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(505_000_000))
    )
    synchronizer._synchronized_images_callback(
        make_image(500_000_000),
        make_image(510_000_000),
    )

    clock[0] += 1_000_000_000
    snapshot = synchronizer.best_snapshot(7, 2)

    assert snapshot[0].header.stamp.nanosec == 500_000_000


def test_collection_uses_tag_observed_during_window(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer.begin_collection(7, 1)
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(100_000_000))
    )
    synchronizer._synchronized_images_callback(
        make_image(100_000_000),
        make_image(105_000_000),
    )
    synchronizer._base_tags_callback(make_tag_array())

    clock[0] += 1_000_000_000

    snapshot = synchronizer.best_snapshot(7, 1)

    assert snapshot is not None
    assert snapshot[0].header.stamp.nanosec == 100_000_000
    assert snapshot[1].header.stamp.nanosec == 105_000_000
    assert snapshot[3].pose.header.stamp.nanosec == 100_000_000


def test_collection_freezes_after_window_end(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer.begin_collection(7, 1)
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(100_000_000))
    )
    synchronizer._synchronized_images_callback(
        make_image(100_000_000),
        make_image(105_000_000),
    )

    clock[0] += 1_100_000_000
    synchronizer._base_tags_callback(
        make_tag_array(make_tag(800_000_000))
    )
    synchronizer._synchronized_images_callback(
        make_image(800_000_000),
        make_image(805_000_000),
    )
    snapshot = synchronizer.best_snapshot(7, 1)

    assert snapshot[0].header.stamp.nanosec == 100_000_000
