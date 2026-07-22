"""Tests for reference-view timestamp candidate selection."""

from collections import deque
from threading import Lock

from fault_detector_msgs.msg import TagElement, TagElementArray
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
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


def make_tag(nanosec):
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.header.stamp.nanosec = nanosec
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_image(nanosec):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    return image


def test_selects_combination_with_smallest_total_skew():
    synchronizer = make_synchronizer()
    image_pairs = [
        (1, make_image(100_000_000), make_image(110_000_000)),
        (2, make_image(500_000_000), make_image(505_000_000)),
    ]
    tags = [
        make_tag(150_000_000),
        make_tag(510_000_000),
    ]

    selected = synchronizer._select_best_candidate(
        image_pairs,
        tags,
    )

    assert selected[0].header.stamp.nanosec == 500_000_000
    assert selected[1].header.stamp.nanosec == 505_000_000
    assert selected[2].pose.header.stamp.nanosec == 510_000_000


def test_rejects_rgb_depth_pair_outside_limit():
    synchronizer = make_synchronizer()
    image_pairs = [
        (1, make_image(100_000_000), make_image(200_000_000)),
    ]

    assert synchronizer._select_best_candidate(
        image_pairs,
        [make_tag(100_000_000)],
    ) is None


def test_rejects_tag_outside_limit():
    synchronizer = make_synchronizer()
    image_pairs = [
        (1, make_image(100_000_000), make_image(105_000_000)),
    ]

    assert synchronizer._select_best_candidate(
        image_pairs,
        [make_tag(500_000_000)],
    ) is None


def test_rejects_zero_timestamps():
    synchronizer = make_synchronizer()
    rgb = Image()
    depth = Image()
    tag = TagElement()

    assert synchronizer._select_best_candidate(
        [(1, rgb, depth)],
        [tag],
    ) is None


def test_out_of_order_tag_does_not_replace_newer_history():
    synchronizer = make_synchronizer()
    observations = TagElementArray()
    observations.elements = [make_tag(400_000_000)]
    synchronizer._base_tags_callback(observations)
    observations.elements = [make_tag(200_000_000)]
    synchronizer._base_tags_callback(observations)

    assert len(synchronizer._base_tags[7]) == 1
    assert (
        synchronizer._base_tags[7][0].pose.header.stamp.nanosec
        == 400_000_000
    )


def test_image_callback_keeps_large_messages_by_reference():
    synchronizer = make_synchronizer()
    rgb = make_image(100_000_000)
    depth = make_image(105_000_000)

    synchronizer._synchronized_images_callback(rgb, depth)

    _, stored_rgb, stored_depth = synchronizer._image_history[-1]
    assert stored_rgb is rgb
    assert stored_depth is depth
