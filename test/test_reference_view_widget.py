"""Tests for the read-only reference-view preview widget."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.widgets import (
    reference_view_widget,
)

ReferenceViewWidget = reference_view_widget.ReferenceViewWidget


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by preview widgets."""
    return QApplication.instance() or QApplication([])


def test_widget_starts_with_empty_state(application):
    """A new widget clearly reports that no image is selected."""
    widget = ReferenceViewWidget()

    assert widget.has_image is False
    assert widget.text() == "No reference view selected"


def test_widget_preserves_source_aspect_ratio(application):
    """Displayed scaling fits the widget without stretching the image."""
    widget = ReferenceViewWidget()
    widget.resize(500, 300)
    widget.set_qimage(QImage(400, 200, QImage.Format_RGB888))

    displayed = widget.pixmap()

    assert widget.has_image is True
    assert displayed is not None
    assert displayed.width() / displayed.height() == pytest.approx(2.0)
    assert displayed.width() <= widget.contentsRect().width()
    assert displayed.height() <= widget.contentsRect().height()


def test_clear_preview_removes_loaded_image(application):
    """Clearing the widget discards the source and shows the reason."""
    widget = ReferenceViewWidget()
    widget.set_qimage(QImage(10, 10, QImage.Format_RGB888))

    widget.clear_preview("No reference view captured")

    assert widget.has_image is False
    assert widget.text() == "No reference view captured"
