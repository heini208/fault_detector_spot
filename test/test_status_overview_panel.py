"""Tests for the compact global status overview panel."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QSizePolicy

from fault_detector_spot.ui.widgets.status_overview_panel import (
    StatusOverviewPanel,
)


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_statuses_share_one_row_and_buffer_uses_second_row(application):
    status = QLabel("Status: Connected")
    command = QLabel("Command: IDLE")
    navigation = QLabel("Navigation: OFF")
    visible = QLabel("Visible tags: []")
    buffer_label = QLabel("Buffer: []")
    estop = QPushButton("EMERGENCY STOP")

    panel = StatusOverviewPanel(
        status,
        command,
        navigation,
        visible,
        buffer_label,
        estop,
    )
    layout = panel.grid_layout

    assert layout.getItemPosition(layout.indexOf(status)) == (0, 0, 1, 1)
    assert layout.getItemPosition(layout.indexOf(command)) == (0, 1, 1, 1)
    assert layout.getItemPosition(layout.indexOf(navigation)) == (
        0,
        2,
        1,
        1,
    )
    assert layout.getItemPosition(layout.indexOf(visible)) == (0, 3, 1, 1)
    assert layout.getItemPosition(layout.indexOf(buffer_label)) == (
        1,
        0,
        1,
        4,
    )
    assert layout.getItemPosition(layout.indexOf(estop)) == (0, 4, 2, 1)
    assert visible.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding
    assert buffer_label.sizePolicy().horizontalPolicy() == (
        QSizePolicy.Expanding
    )
