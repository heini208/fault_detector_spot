"""Tests for exhaustive RGB-depth timestamp matching."""

from collections import deque
from threading import Lock

from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._rgb_camera_info = CameraInfo()
    synchronizer._depth_camera_info = CameraInfo()
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


def test_selects_global_minimum_from_all_raw_combinations():
    synchronizer = make_synchronizer()
    rgb_images = [
        (1, make_image(100_000_000)),
        (2, make_image(150_000_000)),
    ]
    depth_images = [
        (3, make_image(130_000_000)),
        (4, make_image(151_000_000)),
    ]

    selected = synchronizer._select_best_pair(
        rgb_images,
        depth_images,
    )

    assert selected[0].header.stamp.nanosec == 150_000_000
    assert selected[1].header.stamp.nanosec == 151_000_000


def test_rejects_every_pair_outside_limit():
    synchronizer = make_synchronizer()

    assert synchronizer._select_best_pair(
        [(1, make_image(100_000_000))],
        [(2, make_image(200_000_000))],
    ) is None


def test_rejects_zero_timestamps():
    synchronizer = make_synchronizer()

    assert synchronizer._select_best_pair(
        [(1, Image())],
        [(2, Image())],
    ) is None


def test_raw_callbacks_keep_large_messages_by_reference():
    synchronizer = make_synchronizer()
    rgb = make_image(100_000_000)
    depth = make_image(105_000_000)

    synchronizer._rgb_callback(rgb)
    synchronizer._depth_callback(depth)

    assert synchronizer._rgb_history[-1][1] is rgb
    assert synchronizer._depth_history[-1][1] is depth
