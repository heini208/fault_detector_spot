"""Regression tests for processed CameraInfo projection consistency."""

import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)


def make_depth(values, width, height):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.step = width * 4
    image.data = b"".join(struct.pack("<f", value) for value in values)
    return image


def make_camera_info(width, height, focal_length):
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = width
    camera_info.height = height
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    camera_info.k = [
        focal_length,
        0.0,
        cx,
        0.0,
        focal_length,
        cy,
        0.0,
        0.0,
        1.0,
    ]
    camera_info.p = [
        focal_length,
        0.0,
        cx,
        0.0,
        0.0,
        focal_length,
        cy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return camera_info


def test_projection_and_open3d_use_same_processed_camera_model():
    values = [0.0] * 12
    values[2 * 4 + 3] = 2.0
    depth_info = make_camera_info(4, 3, 100.0)
    depth_info.k = [
        50.0,
        0.0,
        0.0,
        0.0,
        50.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    result = project_reference_pixel(
        ImagePoint(u=6, v=4),
        make_depth(values, 4, 3),
        depth_info,
        rgb_size=(8, 6),
        rgb_camera_info=make_camera_info(8, 6, 200.0),
        search_radius_px=0,
    )

    assert result.mapped_pixel == ImagePoint(u=3, v=2)
    assert result.point_camera.x == pytest.approx(0.03)
    assert result.point_camera.y == pytest.approx(0.02)
    assert result.point_camera.z == pytest.approx(2.0)
