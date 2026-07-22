"""Read-only display widget for a captured reference image."""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QFrame, QLabel, QSizePolicy
from sensor_msgs.msg import Image

from fault_detector_spot.inspection.ros_image_conversion import (
    ros_image_to_qimage,
)


class ReferenceViewWidget(QLabel):
    """Display one reference image without changing its aspect ratio."""

    def __init__(self, parent=None):
        """Create an expanding preview with an empty-state message."""
        super().__init__(parent)
        self._source_pixmap = None
        self.setAlignment(Qt.AlignCenter)
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.clear_preview()

    @property
    def has_image(self) -> bool:
        """Return whether a source image is currently loaded."""
        return self._source_pixmap is not None

    def set_ros_image(self, image: Image) -> None:
        """Convert and display a ROS image message."""
        self.set_qimage(ros_image_to_qimage(image))

    def set_qimage(self, image: QImage) -> None:
        """Display an independently owned Qt image."""
        if image.isNull():
            raise ValueError("Reference image must not be null")
        self._source_pixmap = QPixmap.fromImage(image.copy())
        self.setText("")
        self._update_display_pixmap()

    def clear_preview(
        self,
        message: str = "No reference view selected",
    ) -> None:
        """Remove the image and show an empty-state message."""
        self._source_pixmap = None
        self.setPixmap(QPixmap())
        self.setText(message)

    def resizeEvent(self, event) -> None:
        """Rescale the displayed copy when the widget changes size."""
        super().resizeEvent(event)
        self._update_display_pixmap()

    def _update_display_pixmap(self) -> None:
        if self._source_pixmap is None:
            return
        available_size = self.contentsRect().size()
        if available_size.width() <= 0 or available_size.height() <= 0:
            return
        self.setPixmap(self._source_pixmap.scaled(
            available_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        ))
