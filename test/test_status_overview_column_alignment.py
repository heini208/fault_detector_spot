"""Checks that secondary status values align below primary columns."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton

from fault_detector_spot.ui.shared.status_overview_panel import (
    StatusOverviewPanel,
)


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_secondary_statuses_align_under_primary_columns(application):
    status = QLabel("Probe setup opened")
    command = QLabel("Command: IDLE")
    battery_placeholder_navigation = QLabel("Navigation: OFF")
    visible = QLabel("Visible tags: []")
    buffer = QLabel("Buffer: []")

    panel = StatusOverviewPanel(
        status,
        command,
        battery_placeholder_navigation,
        visible,
        buffer,
        QLabel("●"),
        QLabel("hand_closed"),
        QPushButton("✓"),
        QLabel("●"),
        QPushButton("Record"),
        QPushButton("Folder"),
        QLabel("●"),
        QLabel("Offline"),
        QLabel("●"),
        QPushButton("Show IP"),
        QPushButton("Copy"),
        QPushButton("EMERGENCY STOP"),
    )

    grid = panel.grid_layout

    buffer_index = grid.indexOf(buffer)
    visible_index = grid.indexOf(visible)
    navigation_index = grid.indexOf(battery_placeholder_navigation)

    assert grid.getItemPosition(buffer_index)[:2] == (1, 0)
    assert grid.getItemPosition(visible_index)[:2] == (1, 1)
    assert grid.getItemPosition(navigation_index)[:2] == (1, 2)

    panel.close()
