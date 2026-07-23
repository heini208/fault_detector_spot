"""Tests for independent per-camera reference-view synchronization."""

from collections import deque
from threading import Lock

from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
    _CollectionWindow,
)


def make_image(frame_id, nanosec):
    image = Image()
    image.header.frame_id = frame_id
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def make_tag(nanosec):
    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._camera_info = CameraInfo()
    synchronizer._image_sequence = 1
    synchronizer._image_history = deque(maxlen=60)
    synchronizer._tag_history_size = 60
    synchronizer._base_tags = {2: deque([make_tag(110_000_000)])}
    synchronizer._visible_tag_ids = set()
    synchronizer._maximum_timestamp_skew_nanoseconds = 50_000_000
    synchronizer._maximum_tag_timestamp_skew_nanoseconds = 5_000_000_000
    synchronizer._collection_duration_nanoseconds = 1_000_000_000
    synchronizer._collection = None
    return synchronizer


def test_snapshot_uses_tag_captured_in_window_after_visibility_drops():
    synchronizer = make_synchronizer()
    rgb = make_image("hand_color_image_sensor", 100_000_000)
    depth = make_image("hand_color_image_sensor", 105_000_000)
    synchronizer._collection = _CollectionWindow(
        reference_tag_id=2,
        minimum_image_sequence=1,
        ends_at_nanoseconds=0,
        image_pairs=deque([(1, rgb, depth)]),
        tag_observations=deque([make_tag(110_000_000)]),
    )

    snapshot = synchronizer.best_snapshot(2, 1)

    assert snapshot is not None
    assert snapshot[0].header.frame_id == "hand_color_image_sensor"
    assert snapshot[3].id == 2


def test_ready_for_collection_requires_camera_pair_info_and_tag_history():
    synchronizer = make_synchronizer()

    assert synchronizer.ready_for_collection(2) is True

    synchronizer._camera_info = None
    assert synchronizer.ready_for_collection(2) is False


def test_collection_diagnostics_are_camera_specific():
    synchronizer = make_synchronizer()
    synchronizer._collection = _CollectionWindow(
        reference_tag_id=2,
        minimum_image_sequence=2,
        ends_at_nanoseconds=0,
        image_pairs=deque(),
        tag_observations=deque([make_tag(110_000_000)]),
    )

    diagnostics = synchronizer.collection_diagnostics(2, 2)

    assert "rgb_depth_pairs=0" in diagnostics
    assert "tag_samples=1" in diagnostics
    assert "tag_visible_at_end=no" in diagnostics
