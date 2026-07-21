"""Tests for timestamp-aware reference-view tag selection."""

from collections import deque
from threading import Lock

from fault_detector_msgs.msg import TagElement, TagElementArray
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


def make_synchronizer() -> ReferenceViewInputSynchronizer:
    synchronizer = ReferenceViewInputSynchronizer.__new__(
        ReferenceViewInputSynchronizer
    )
    synchronizer._lock = Lock()
    synchronizer._camera_info = CameraInfo()
    synchronizer._latest_images = None
    synchronizer._image_sequence = 0
    synchronizer._tag_history_size = 10
    synchronizer._base_tags = {}
    synchronizer._visible_tag_ids = set()
    return synchronizer


def make_tag(nanosec: int) -> TagElement:
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_image(nanosec: int) -> Image:
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def test_snapshot_selects_tag_nearest_to_rgb_timestamp():
    synchronizer = make_synchronizer()
    observations = TagElementArray()
    observations.elements = [
        make_tag(100_000_000),
        make_tag(400_000_000),
    ]
    synchronizer._base_tags_callback(observations)
    rgb = make_image(320_000_000)
    depth = make_image(325_000_000)
    synchronizer._synchronized_images_callback(rgb, depth)

    snapshot = synchronizer.snapshot(7)

    assert snapshot[3].pose.header.stamp.nanosec == 400_000_000


def test_out_of_order_tag_does_not_replace_newer_history():
    synchronizer = make_synchronizer()
    observations = TagElementArray()
    observations.elements = [make_tag(400_000_000)]
    synchronizer._base_tags_callback(observations)
    observations.elements = [make_tag(200_000_000)]
    synchronizer._base_tags_callback(observations)
    synchronizer._synchronized_images_callback(
        make_image(210_000_000),
        make_image(215_000_000),
    )

    snapshot = synchronizer.snapshot(7)

    assert snapshot[3].pose.header.stamp.nanosec == 400_000_000
    assert len(synchronizer._base_tags[7]) == 1
    assert isinstance(synchronizer._base_tags[7], deque)


def test_image_callback_does_not_copy_full_image_messages():
    synchronizer = make_synchronizer()
    rgb = make_image(100_000_000)
    depth = make_image(110_000_000)

    synchronizer._synchronized_images_callback(rgb, depth)

    assert synchronizer._latest_images[0] is rgb
    assert synchronizer._latest_images[1] is depth
