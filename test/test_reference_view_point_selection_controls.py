"""Tests for inspection-control point-selection feedback."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import ProbeSetupIntent
from PyQt5.QtCore import QEvent, QPointF, Qt
from PyQt5.QtGui import QImage, QMouseEvent
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.ui.inspection.controls import (
    InspectionControls,
)


class FakeUI:
    """Provide the parent contract used by inspection controls."""

    def __init__(self, object_root):
        """Create isolated UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.requests = []

    def execute_probe_setup(self, intent):
        self.requests.append(intent)
        return "request"

    def show_setup_unavailable(self, workflow):
        return False


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
    controls._reference_slot_view_ids[0] = "slot1_hand"
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
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_SELECT_REFERENCE_PIXEL
    )
    assert controls.ui.requests[-1].reference_view_id == "slot1_hand"

    controls.clear_reference_pixel_button.click()

    assert widget.selected_image_point is None
    assert controls.reference_pixel_value_label.text() == "—"
    assert controls.clear_reference_pixel_button.isEnabled() is False
    assert controls.ui.requests[-1].operation == (
        ProbeSetupIntent.OPERATION_CLEAR_REFERENCE_PIXEL
    )
