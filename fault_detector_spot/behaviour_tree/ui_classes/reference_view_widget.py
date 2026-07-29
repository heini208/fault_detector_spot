"""Interactive display widget for a captured reference image."""

from typing import Optional

from PyQt5.QtCore import QPointF, QRect, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QFrame, QLabel, QSizePolicy
from sensor_msgs.msg import Image

from fault_detector_spot.inspection.models import ImagePoint
from fault_detector_spot.inspection.reference_view_depth_projection import (
    ImageRegion,
)
from fault_detector_spot.inspection.ros_image_conversion import (
    ros_image_to_qimage,
)


class ReferenceViewWidget(QLabel):
    """Display a reference image and select one source-image pixel."""

    image_point_changed = pyqtSignal(int, int)
    image_point_cleared = pyqtSignal()

    def __init__(self, parent=None):
        """Create an expanding preview with transient point selection."""
        super().__init__(parent)
        self._source_pixmap = None
        self._source_offset = ImagePoint(u=0, v=0)
        self._displayed_image_rect = QRect()
        self._selected_image_point = None
        self._dragging_marker = False
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

    @property
    def selected_image_point(self) -> Optional[ImagePoint]:
        """Return an isolated copy of the selected source-image pixel."""
        if self._selected_image_point is None:
            return None
        return ImagePoint(
            u=self._selected_image_point.u,
            v=self._selected_image_point.v,
        )

    @property
    def displayed_image_rect(self) -> QRect:
        """Return the widget rectangle occupied by the scaled image."""
        return QRect(self._displayed_image_rect)

    def set_ros_image(
        self,
        image: Image,
        valid_region: Optional[ImageRegion] = None,
    ) -> None:
        """Convert and display a ROS image message."""
        self.set_qimage(
            ros_image_to_qimage(image),
            valid_region=valid_region,
        )

    def set_qimage(
        self,
        image: QImage,
        valid_region: Optional[ImageRegion] = None,
    ) -> None:
        """Display an independently owned Qt image and clear selection."""
        if image.isNull():
            raise ValueError("Reference image must not be null")
        source = image
        offset = ImagePoint(u=0, v=0)
        if valid_region is not None:
            valid_region.validate()
            if (
                valid_region.x + valid_region.width > image.width()
                or valid_region.y + valid_region.height > image.height()
            ):
                raise ValueError(
                    "Reference image region exceeds the source image"
                )
            source = image.copy(
                valid_region.x,
                valid_region.y,
                valid_region.width,
                valid_region.height,
            )
            offset = ImagePoint(
                u=valid_region.x,
                v=valid_region.y,
            )
        self.clear_selection()
        self._source_offset = offset
        self._source_pixmap = QPixmap.fromImage(source.copy())
        self.setText("")
        self.setCursor(Qt.CrossCursor)
        self._update_display_pixmap()

    def clear_preview(
        self,
        message: str = "No reference view selected",
    ) -> None:
        """Remove the image, selection, and show an empty-state message."""
        self.clear_selection()
        self._source_pixmap = None
        self._source_offset = ImagePoint(u=0, v=0)
        self._displayed_image_rect = QRect()
        self.setPixmap(QPixmap())
        self.setText(message)
        self.unsetCursor()

    def clear_selection(self) -> None:
        """Remove the transient point selection."""
        had_selection = self._selected_image_point is not None
        self._selected_image_point = None
        self._dragging_marker = False
        self.update()
        if had_selection:
            self.image_point_cleared.emit()

    def resizeEvent(self, event) -> None:
        """Rescale the displayed copy when the widget changes size."""
        super().resizeEvent(event)
        self._update_display_pixmap()

    def paintEvent(self, event) -> None:
        """Draw the selected point over the scaled reference image."""
        super().paintEvent(event)
        marker_center = self._selected_marker_center()
        if marker_center is None:
            return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(255, 64, 64), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(marker_center, 8.0, 8.0)
        painter.drawLine(
            QPointF(marker_center.x() - 11.0, marker_center.y()),
            QPointF(marker_center.x() + 11.0, marker_center.y()),
        )
        painter.drawLine(
            QPointF(marker_center.x(), marker_center.y() - 11.0),
            QPointF(marker_center.x(), marker_center.y() + 11.0),
        )
        painter.end()

    def mousePressEvent(self, event) -> None:
        """Place a marker when the left button is pressed on the image."""
        if event.button() == Qt.LeftButton:
            point = self._widget_to_image_point(event.pos(), clamp=False)
            if point is not None:
                self._dragging_marker = True
                self._set_selected_image_point(point)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        """Move the marker while dragging, clamped to image bounds."""
        if self._dragging_marker and event.buttons() & Qt.LeftButton:
            point = self._widget_to_image_point(event.pos(), clamp=True)
            if point is not None:
                self._set_selected_image_point(point)
                event.accept()
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        """Finish one marker drag on left-button release."""
        if event.button() == Qt.LeftButton and self._dragging_marker:
            point = self._widget_to_image_point(event.pos(), clamp=True)
            if point is not None:
                self._set_selected_image_point(point)
            self._dragging_marker = False
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _set_selected_image_point(self, point: ImagePoint) -> None:
        if self._selected_image_point == point:
            return
        self._selected_image_point = point
        self.update()
        self.image_point_changed.emit(point.u, point.v)

    def _widget_to_image_point(
        self,
        widget_point,
        clamp: bool,
    ) -> Optional[ImagePoint]:
        if self._source_pixmap is None or self._displayed_image_rect.isEmpty():
            return None

        image_rect = self._displayed_image_rect
        x = widget_point.x()
        y = widget_point.y()
        inside = (
            image_rect.left() <= x < image_rect.left() + image_rect.width()
            and image_rect.top() <= y < image_rect.top() + image_rect.height()
        )
        if not inside and not clamp:
            return None

        x = min(
            max(x, image_rect.left()),
            image_rect.left() + image_rect.width() - 1,
        )
        y = min(
            max(y, image_rect.top()),
            image_rect.top() + image_rect.height() - 1,
        )
        relative_x = x - image_rect.left()
        relative_y = y - image_rect.top()
        source_width = self._source_pixmap.width()
        source_height = self._source_pixmap.height()
        u = min(
            source_width - 1,
            int(relative_x * source_width / image_rect.width()),
        ) + self._source_offset.u
        v = min(
            source_height - 1,
            int(relative_y * source_height / image_rect.height()),
        ) + self._source_offset.v
        return ImagePoint(u=u, v=v)

    def _selected_marker_center(self) -> Optional[QPointF]:
        if (
            self._source_pixmap is None
            or self._selected_image_point is None
            or self._displayed_image_rect.isEmpty()
        ):
            return None
        image_rect = self._displayed_image_rect
        source_width = self._source_pixmap.width()
        source_height = self._source_pixmap.height()
        x = image_rect.left() + (
            (
                self._selected_image_point.u
                - self._source_offset.u
                + 0.5
            )
            * image_rect.width()
            / source_width
        )
        y = image_rect.top() + (
            (
                self._selected_image_point.v
                - self._source_offset.v
                + 0.5
            )
            * image_rect.height()
            / source_height
        )
        return QPointF(x, y)

    def _update_display_pixmap(self) -> None:
        if self._source_pixmap is None:
            self._displayed_image_rect = QRect()
            return
        contents = self.contentsRect()
        available_size = contents.size()
        if available_size.width() <= 0 or available_size.height() <= 0:
            self._displayed_image_rect = QRect()
            return
        displayed = self._source_pixmap.scaled(
            available_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.setPixmap(displayed)
        left = contents.left() + (
            contents.width() - displayed.width()
        ) // 2
        top = contents.top() + (
            contents.height() - displayed.height()
        ) // 2
        self._displayed_image_rect = QRect(
            left,
            top,
            displayed.width(),
            displayed.height(),
        )
        self.update()
