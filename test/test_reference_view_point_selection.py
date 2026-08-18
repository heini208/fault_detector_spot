"""Tests for transient reference-image point selection."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtCore import QEvent, QPoint, QPointF, Qt
from PyQt5.QtGui import QImage, QMouseEvent
from PyQt5.QtWidgets import QApplication, QFrame

from fault_detector_spot.ui.inspection import (
    reference_view_widget,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)
from fault_detector_spot.inspection.model.models import ImagePoint

ReferenceViewWidget = reference_view_widget.ReferenceViewWidget


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by interactive widgets."""
    return QApplication.instance() or QApplication([])


def make_widget(application, width=500, height=300):
    """Create one visible widget with a 400 by 200 source image."""
    widget = ReferenceViewWidget()
    widget.setFrameShape(QFrame.NoFrame)
    widget.resize(width, height)
    widget.set_qimage(QImage(400, 200, QImage.Format_RGB888))
    widget.show()
    application.processEvents()
    return widget


def send_mouse(widget, event_type, position, button, buttons):
    """Send one explicit mouse event to the widget."""
    event = QMouseEvent(
        event_type,
        QPointF(position),
        button,
        buttons,
        Qt.NoModifier,
    )
    QApplication.sendEvent(widget, event)


def source_pixel_position(widget, u, v):
    """Return a widget position centered over one source pixel."""
    rect = widget.displayed_image_rect
    return QPoint(
        int(rect.left() + (u + 0.5) * rect.width() / 400),
        int(rect.top() + (v + 0.5) * rect.height() / 200),
    )


def select_pixel(widget, u, v):
    """Click one source pixel in the displayed image."""
    position = source_pixel_position(widget, u, v)
    send_mouse(
        widget,
        QEvent.MouseButtonPress,
        position,
        Qt.LeftButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseButtonRelease,
        position,
        Qt.LeftButton,
        Qt.NoButton,
    )


def test_click_selects_native_image_pixel(application):
    """A displayed click is reported in original image coordinates."""
    widget = make_widget(application)
    changed = []
    widget.image_point_changed.connect(
        lambda u, v: changed.append((u, v))
    )

    select_pixel(widget, 123, 87)

    point = widget.selected_image_point
    assert point is not None
    assert point.u == pytest.approx(123, abs=1)
    assert point.v == pytest.approx(87, abs=1)
    assert changed == [(point.u, point.v)]


def test_click_in_letterbox_margin_is_ignored(application):
    """Empty space around the scaled image cannot create a point."""
    widget = make_widget(application, width=400, height=400)
    image_rect = widget.displayed_image_rect
    margin_position = QPoint(
        image_rect.center().x(),
        image_rect.top() - 10,
    )

    send_mouse(
        widget,
        QEvent.MouseButtonPress,
        margin_position,
        Qt.LeftButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseButtonRelease,
        margin_position,
        Qt.LeftButton,
        Qt.NoButton,
    )

    assert widget.selected_image_point is None


def test_resize_preserves_native_image_pixel(application):
    """Widget scaling never changes the selected source coordinate."""
    widget = make_widget(application)
    select_pixel(widget, 210, 45)
    selected_before = widget.selected_image_point

    widget.resize(800, 250)
    application.processEvents()

    assert widget.selected_image_point == selected_before


def test_dragging_clamps_marker_to_image_bounds(application):
    """Dragging outside the image stops at the final source pixel."""
    widget = make_widget(application)
    start = source_pixel_position(widget, 200, 100)
    outside = QPoint(
        widget.displayed_image_rect.right() + 200,
        widget.displayed_image_rect.bottom() + 200,
    )

    send_mouse(
        widget,
        QEvent.MouseButtonPress,
        start,
        Qt.LeftButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseMove,
        outside,
        Qt.NoButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseButtonRelease,
        outside,
        Qt.LeftButton,
        Qt.NoButton,
    )

    point = widget.selected_image_point
    assert point is not None
    assert point.u == 399
    assert point.v == 199


def test_dragging_emits_only_final_committed_pixel(application):
    widget = make_widget(application)
    changed = []
    widget.image_point_changed.connect(
        lambda u, v: changed.append((u, v))
    )
    start = source_pixel_position(widget, 50, 50)
    middle = source_pixel_position(widget, 150, 75)
    final = source_pixel_position(widget, 250, 100)

    send_mouse(
        widget,
        QEvent.MouseButtonPress,
        start,
        Qt.LeftButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseMove,
        middle,
        Qt.NoButton,
        Qt.LeftButton,
    )
    assert changed == []

    send_mouse(
        widget,
        QEvent.MouseMove,
        final,
        Qt.NoButton,
        Qt.LeftButton,
    )
    assert changed == []

    send_mouse(
        widget,
        QEvent.MouseButtonRelease,
        final,
        Qt.LeftButton,
        Qt.NoButton,
    )

    point = widget.selected_image_point
    assert changed == [(point.u, point.v)]


def test_programmatic_selection_preserves_change_signal(application):
    widget = make_widget(application)
    changed = []
    widget.image_point_changed.connect(
        lambda u, v: changed.append((u, v))
    )

    widget.set_selected_image_point(ImagePoint(u=80, v=60))

    assert widget.selected_image_point == ImagePoint(u=80, v=60)
    assert changed == [(80, 60)]


def test_clear_selection_emits_and_removes_marker(application):
    """Explicit clearing resets the transient selection state."""
    widget = make_widget(application)
    cleared = []
    widget.image_point_cleared.connect(lambda: cleared.append(True))
    select_pixel(widget, 20, 30)

    widget.clear_selection()

    assert widget.selected_image_point is None
    assert cleared == [True]


def test_loading_another_image_clears_selection(application):
    """A point cannot carry over to another reference image."""
    widget = make_widget(application)
    select_pixel(widget, 20, 30)

    widget.set_qimage(QImage(200, 100, QImage.Format_RGB888))

    assert widget.selected_image_point is None


def test_cropped_preview_reports_original_rgb_coordinates(application):
    widget = ReferenceViewWidget()
    widget.setFrameShape(QFrame.NoFrame)
    widget.resize(500, 300)
    widget.set_qimage(
        QImage(400, 200, QImage.Format_RGB888),
        valid_region=ImageRegion(
            x=100,
            y=50,
            width=200,
            height=100,
        ),
    )
    widget.show()
    application.processEvents()
    rect = widget.displayed_image_rect
    position = QPoint(
        int(rect.left() + 0.5 * rect.width() / 200),
        int(rect.top() + 0.5 * rect.height() / 100),
    )

    send_mouse(
        widget,
        QEvent.MouseButtonPress,
        position,
        Qt.LeftButton,
        Qt.LeftButton,
    )
    send_mouse(
        widget,
        QEvent.MouseButtonRelease,
        position,
        Qt.LeftButton,
        Qt.NoButton,
    )

    assert widget.selected_image_point == ImagePoint(u=100, v=50)
