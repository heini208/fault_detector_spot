"""Tests for direct shared-tag injection into camera synchronizers."""

from collections import deque
from threading import Lock

from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo

from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._camera_info = CameraInfo()
    synchronizer._image_sequence = 1
    synchronizer._image_history = deque(maxlen=60)
    synchronizer._tag_history_size = 60
    synchronizer._base_tags = {}
    synchronizer._visible_tag_ids = set()
    synchronizer._maximum_timestamp_skew_nanoseconds = 50_000_000
    synchronizer._maximum_tag_timestamp_skew_nanoseconds = 250_000_000
    synchronizer._collection = None
    return synchronizer


def test_direct_tag_snapshot_warms_camera_synchronizer():
    synchronizer = make_synchronizer()
    tag = TagElement()
    tag.id = 7
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.orientation.w = 1.0

    synchronizer.update_base_tag_observations([tag])

    assert synchronizer.ready_for_collection(7) is True
    assert synchronizer.input_diagnostics(7).startswith(
        "camera_info=yes, rgb_depth_pairs=0, tag_samples=1"
    )


def test_empty_snapshot_does_not_delete_collected_tag_history():
    synchronizer = make_synchronizer()
    tag = TagElement()
    tag.id = 7
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.orientation.w = 1.0

    synchronizer.update_base_tag_observations([tag])
    synchronizer.update_base_tag_observations([])

    assert synchronizer.ready_for_collection(7) is True
    assert "tag_visible_at_end=no" in synchronizer.input_diagnostics(7)
