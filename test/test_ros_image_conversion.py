"""Tests for ROS-to-Qt image conversion."""

import pytest
from PyQt5.QtGui import QColor
from sensor_msgs.msg import Image

from fault_detector_spot.inspection.ros_image_conversion import (
    ros_image_to_qimage,
)


def make_image(encoding, data, width=2, height=1, step=None):
    """Create one small ROS image with explicit byte storage."""
    bytes_per_pixel = {
        "rgb8": 3,
        "bgr8": 3,
        "mono8": 1,
    }.get(encoding, 1)
    image = Image()
    image.width = width
    image.height = height
    image.encoding = encoding
    image.step = step if step is not None else width * bytes_per_pixel
    image.data = bytes(data)
    return image


def test_rgb8_conversion_preserves_pixel_channels():
    """RGB bytes retain their channel meaning in the Qt image."""
    image = make_image(
        "rgb8",
        [255, 0, 0, 0, 255, 0],
    )

    converted = ros_image_to_qimage(image)

    assert converted.pixelColor(0, 0) == QColor(255, 0, 0)
    assert converted.pixelColor(1, 0) == QColor(0, 255, 0)


def test_bgr8_conversion_swaps_red_and_blue_channels():
    """BGR camera bytes are displayed with correct RGB colors."""
    image = make_image(
        "bgr8",
        [0, 0, 255, 255, 0, 0],
    )

    converted = ros_image_to_qimage(image)

    assert converted.pixelColor(0, 0) == QColor(255, 0, 0)
    assert converted.pixelColor(1, 0) == QColor(0, 0, 255)


def test_conversion_copies_message_storage():
    """The Qt image remains valid after the ROS message changes."""
    image = make_image("rgb8", [255, 0, 0, 0, 255, 0])
    converted = ros_image_to_qimage(image)

    image.data = bytes([0] * 6)

    assert converted.pixelColor(0, 0) == QColor(255, 0, 0)


@pytest.mark.parametrize(
    "image, message",
    [
        (make_image("16UC1", [0, 0, 0, 0]), "Unsupported"),
        (make_image("rgb8", [0] * 6, width=0), "dimensions"),
        (make_image("rgb8", [0] * 5), "shorter"),
        (make_image("rgb8", [0] * 6, step=5), "step"),
    ],
)
def test_invalid_images_are_rejected(image, message):
    """Malformed and unsupported messages fail with useful errors."""
    with pytest.raises(ValueError, match=message):
        ros_image_to_qimage(image)
