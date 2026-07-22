"""Convert ROS image messages into detached Qt images."""

from PyQt5.QtGui import QImage
from sensor_msgs.msg import Image


_IMAGE_FORMATS = {
    "rgb8": (3, QImage.Format_RGB888, False),
    "bgr8": (3, QImage.Format_RGB888, True),
    "mono8": (1, QImage.Format_Grayscale8, False),
}


def ros_image_to_qimage(image: Image) -> QImage:
    """Convert one supported ROS image into independently owned Qt data."""
    if image.width <= 0 or image.height <= 0:
        raise ValueError("Image dimensions must be positive")

    encoding = image.encoding.strip().lower()
    if encoding not in _IMAGE_FORMATS:
        raise ValueError(f"Unsupported image encoding: {image.encoding}")

    bytes_per_pixel, image_format, swap_red_blue = _IMAGE_FORMATS[
        encoding
    ]
    minimum_step = image.width * bytes_per_pixel
    if image.step < minimum_step:
        raise ValueError(
            f"Image step is too small for {encoding}: {image.step}"
        )

    required_size = image.step * image.height
    data = bytes(image.data)
    if len(data) < required_size:
        raise ValueError(
            "Image data is shorter than step multiplied by height"
        )

    qt_image = QImage(
        data,
        image.width,
        image.height,
        image.step,
        image_format,
    )
    if qt_image.isNull():
        raise ValueError("Image data could not be converted")
    if swap_red_blue:
        qt_image = qt_image.rgbSwapped()
    return qt_image.copy()
