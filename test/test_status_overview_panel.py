"""Tests for the compact global status overview panel."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QSizePolicy

from fault_detector_spot.ui.shared.status_overview_panel import (
    StatusOverviewPanel,
)


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_primary_statuses_use_first_row_and_context_uses_second_row(
    application,
):
    status = QLabel("Status: Connected")
    command = QLabel("Command: IDLE")
    navigation = QLabel("Navigation: OFF")
    visible = QLabel("Visible tags: []")
    buffer_label = QLabel("Buffer: []")
    sensor_indicator = QLabel("●")
    sensor = QLabel("No sensor")
    confirm = QPushButton("✓")
    estop = QPushButton("EMERGENCY STOP")

    panel = StatusOverviewPanel(
        status,
        command,
        navigation,
        visible,
        buffer_label,
        sensor_indicator,
        sensor,
        confirm,
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
    assert layout.getItemPosition(layout.indexOf(estop)) == (0, 3, 2, 1)

    assert layout.getItemPosition(layout.indexOf(buffer_label)) == (
        1,
        0,
        1,
        1,
    )
    assert layout.getItemPosition(layout.indexOf(visible)) == (
        1,
        1,
        1,
        1,
    )
    assert layout.getItemPosition(layout.indexOf(panel.sensor_widget)) == (
        1,
        2,
        1,
        1,
    )
    assert panel.sensor_widget.layout().indexOf(sensor_indicator) >= 0
    assert panel.sensor_widget.layout().indexOf(sensor) >= 0
    assert panel.sensor_widget.layout().indexOf(confirm) >= 0

    assert buffer_label.sizePolicy().horizontalPolicy() == (
        QSizePolicy.Expanding
    )
    assert visible.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding
    assert sensor.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding
