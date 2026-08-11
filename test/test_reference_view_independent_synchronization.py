"""Tests for independent camera collection state."""

from collections import deque
from threading import Lock

from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
    _CollectionWindow,
)


def make_image(frame_id, nanosec):
    image = Image()
    image.header.frame_id = frame_id
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._rgb_camera_info = CameraInfo()
    synchronizer._depth_camera_info = CameraInfo()
    synchronizer._input_sequence = 2
    synchronizer._rgb_history = deque(
        [(1, make_image("camera", 100_000_000))],
        maxlen=2,
    )
    synchronizer._depth_history = deque(
        [(2, make_image("camera", 105_000_000))],
        maxlen=2,
    )
    synchronizer._maximum_timestamp_skew_nanoseconds = 50_000_000
    synchronizer._collection_duration_nanoseconds = 1_000_000_000
    synchronizer._collection_history_size = 60
    synchronizer._collection = None
    return synchronizer


def test_ready_requires_both_images_and_both_calibrations():
    synchronizer = make_synchronizer()

    assert synchronizer.ready_for_collection() is True

    synchronizer._rgb_camera_info = None
    assert synchronizer.ready_for_collection() is False


def test_collection_diagnostics_are_camera_specific():
    synchronizer = make_synchronizer()
    synchronizer._collection = _CollectionWindow(
        minimum_input_sequence=3,
        ends_at_nanoseconds=0,
        rgb_images=deque(),
        depth_images=deque(),
    )

    diagnostics = synchronizer.collection_diagnostics(3)

    assert "rgb_frames=0" in diagnostics
    assert "depth_frames=0" in diagnostics
    assert "valid_pair=no" in diagnostics
