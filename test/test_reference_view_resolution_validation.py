"""Tests for reference capture with different registered resolutions."""

from fault_detector_msgs.msg import TagElement
import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.models import PoseData
from fault_detector_spot.inspection.reference_view_validation import (
    validate_reference_view_inputs,
)


def make_inputs():
    """Create synchronized RGB and lower-resolution registered depth."""
    rgb = Image()
    rgb.header.frame_id = "hand_color_image_sensor"
    rgb.header.stamp.sec = 10
    rgb.width = 8
    rgb.height = 6
    rgb.encoding = "rgb8"
    rgb.step = 24
    rgb.data = bytes(rgb.step * rgb.height)

    depth = Image()
    depth.header.frame_id = "hand_color_image_sensor"
    depth.header.stamp.sec = 10
    depth.header.stamp.nanosec = 10_000_000
    depth.width = 4
    depth.height = 3
    depth.encoding = "16UC1"
    depth.step = 8
    depth.data = bytes(depth.step * depth.height)

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 4
    camera_info.height = 3
    camera_info.k = [
        100.0,
        0.0,
        1.5,
        0.0,
        100.0,
        1.0,
        0.0,
        0.0,
        1.0,
    ]

    tag = TagElement()
    tag.id = 23
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.orientation.w = 1.0
    return rgb, depth, camera_info, tag


def test_registered_depth_may_use_a_different_resolution():
    """Only depth and its CameraInfo must share dimensions."""
    rgb, depth, camera_info, tag = make_inputs()

    validate_reference_view_inputs(
        rgb,
        depth,
        camera_info,
        tag,
        expected_reference_tag_id=23,
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
    )


def test_depth_and_camera_info_dimensions_still_must_match():
    """Back-projection cannot use calibration from another depth size."""
    rgb, depth, camera_info, tag = make_inputs()
    camera_info.width = 5

    with pytest.raises(ValueError, match="Registered depth"):
        validate_reference_view_inputs(
            rgb,
            depth,
            camera_info,
            tag,
            expected_reference_tag_id=23,
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_color_image_sensor",
        )
