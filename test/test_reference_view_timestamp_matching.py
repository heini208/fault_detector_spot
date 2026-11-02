"""Tests for RGB-depth and timestamp-anchor matching."""

from unittest.mock import Mock

from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


def make_synchronizer():
    synchronizer = ReferenceViewInputSynchronizer(
        node=Mock(spec=["create_subscription"]),
        rgb_topic="/camera/hand/image",
        depth_topic="/depth_registered/hand/image",
        rgb_camera_info_topic="/camera/hand/camera_info",
        depth_camera_info_topic="/depth_registered/hand/camera_info",
        maximum_timestamp_skew_sec=0.05,
        collection_duration_sec=1.0,
    )
    synchronizer._rgb_camera_info_callback(CameraInfo())
    synchronizer._depth_camera_info_callback(CameraInfo())
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


def test_triplet_matching_can_choose_a_less_exact_rgb_depth_pair():
    synchronizer = make_synchronizer()
    rgb_images = [
        (1, make_image(100_000_000)),
        (2, make_image(145_000_000)),
    ]
    depth_images = [
        (3, make_image(101_000_000)),
        (4, make_image(149_000_000)),
    ]

    selected = synchronizer._select_best_triplet(
        rgb_images,
        depth_images,
        (10_152_000_000,),
    )

    assert selected[0].header.stamp.nanosec == 145_000_000
    assert selected[1].header.stamp.nanosec == 149_000_000
    assert selected[2] == 0


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
