"""Tests for inspection-control point-selection feedback."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtCore import QEvent, QPointF, Qt
from PyQt5.QtGui import QImage, QMouseEvent
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)


class FakePublisher:
    """Accept commands without affecting point-selection tests."""

    def publish(self, message):
        """Discard one published command."""


class FakeUI:
    """Provide the parent contract used by inspection controls."""

    def __init__(self, object_root):
        """Create isolated UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root

    def build_basic_command(self, command_id):
        """Build a minimal command header."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by controls."""
    return QApplication.instance() or QApplication([])


def test_controls_report_and_clear_selected_pixel(
    application,
    tmp_path,
):
    """The setup UI mirrors the widget's transient selected coordinate."""
    controls = InspectionControls(FakeUI(tmp_path))
    widget = controls.reference_view_widget
    widget.resize(400, 240)
    widget.set_qimage(QImage(400, 200, QImage.Format_RGB888))
    widget.show()
    application.processEvents()
    image_rect = widget.displayed_image_rect
    position = image_rect.center()
    press = QMouseEvent(
        QEvent.MouseButtonPress,
        QPointF(position),
        Qt.LeftButton,
        Qt.LeftButton,
        Qt.NoModifier,
    )
    release = QMouseEvent(
        QEvent.MouseButtonRelease,
        QPointF(position),
        Qt.LeftButton,
        Qt.NoButton,
        Qt.NoModifier,
    )

    QApplication.sendEvent(widget, press)
    QApplication.sendEvent(widget, release)

    point = widget.selected_image_point
    assert point is not None
    assert controls.reference_pixel_value_label.text() == (
        f"u={point.u}, v={point.v}"
    )
    assert controls.clear_reference_pixel_button.isEnabled() is True

    controls.clear_reference_pixel_button.click()

    assert widget.selected_image_point is None
    assert controls.reference_pixel_value_label.text() == "—"
    assert controls.clear_reference_pixel_button.isEnabled() is False
