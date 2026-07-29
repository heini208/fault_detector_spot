"""Tests for bounded raw camera collection."""

from collections import deque
from threading import Lock

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import (
    reference_view_input_synchronizer as synchronizer_module,
)


ReferenceViewInputSynchronizer = (
    synchronizer_module.ReferenceViewInputSynchronizer
)


def make_camera_info():
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 1
    camera_info.height = 1
    camera_info.k = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    return camera_info


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._rgb_camera_info = make_camera_info()
    synchronizer._depth_camera_info = make_camera_info()
    synchronizer._input_sequence = 0
    synchronizer._collection_history_size = 60
    synchronizer._rgb_history = deque(maxlen=2)
    synchronizer._depth_history = deque(maxlen=2)
    synchronizer._maximum_timestamp_skew_nanoseconds = 50_000_000
    synchronizer._collection_duration_nanoseconds = 1_000_000_000
    synchronizer._collection = None
    return synchronizer


def make_image(nanosec):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def test_collection_waits_one_second_then_selects_best_pair(
    monkeypatch,
):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer.begin_collection(1)
    synchronizer._rgb_callback(make_image(100_000_000))
    synchronizer._depth_callback(make_image(110_000_000))
    synchronizer._rgb_callback(make_image(500_000_000))
    synchronizer._depth_callback(make_image(505_000_000))

    assert synchronizer.collection_ready() is False
    with pytest.raises(RuntimeError, match="not complete"):
        synchronizer.best_snapshot(1)

    clock[0] += 1_000_000_000
    snapshot = synchronizer.best_snapshot(1)

    assert snapshot[0].header.stamp.nanosec == 500_000_000
    assert snapshot[1].header.stamp.nanosec == 505_000_000


def test_collection_excludes_inputs_before_minimum_sequence(
    monkeypatch,
):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer._rgb_callback(make_image(100_000_000))
    synchronizer._depth_callback(make_image(105_000_000))
    synchronizer.begin_collection(3)
    synchronizer._rgb_callback(make_image(500_000_000))
    synchronizer._depth_callback(make_image(510_000_000))

    clock[0] += 1_000_000_000
    snapshot = synchronizer.best_snapshot(3)

    assert snapshot[0].header.stamp.nanosec == 500_000_000


def test_collection_freezes_after_window_end(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer.begin_collection(1)
    synchronizer._rgb_callback(make_image(100_000_000))
    synchronizer._depth_callback(make_image(105_000_000))

    clock[0] += 1_100_000_000
    synchronizer._rgb_callback(make_image(800_000_000))
    synchronizer._depth_callback(make_image(805_000_000))
    snapshot = synchronizer.best_snapshot(1)

    assert snapshot[0].header.stamp.nanosec == 100_000_000


def test_collection_freezes_both_calibrations_at_start(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr(
        synchronizer_module.time,
        "monotonic_ns",
        lambda: clock[0],
    )
    synchronizer = make_synchronizer()
    synchronizer._rgb_camera_info.k[0] = 100.0
    synchronizer._depth_camera_info.k[0] = 200.0
    synchronizer.begin_collection(1)

    newer_rgb_info = make_camera_info()
    newer_depth_info = make_camera_info()
    newer_rgb_info.k[0] = 300.0
    newer_depth_info.k[0] = 400.0
    synchronizer._rgb_camera_info_callback(newer_rgb_info)
    synchronizer._depth_camera_info_callback(newer_depth_info)
    synchronizer._rgb_callback(make_image(100_000_000))
    synchronizer._depth_callback(make_image(105_000_000))

    clock[0] += 1_000_000_000
    snapshot = synchronizer.best_snapshot(1)

    assert snapshot[2].k[0] == 100.0
    assert snapshot[3].k[0] == 200.0
