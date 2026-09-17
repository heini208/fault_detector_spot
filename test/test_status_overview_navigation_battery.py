"""Checks for navigation and battery placement in the status overview."""

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


def make_panel():
    navigation = QLabel("Navigation: OFF")
    panel = StatusOverviewPanel(
        QLabel("Status"),
        QLabel("Command: IDLE"),
        navigation,
        QLabel("Visible tags: []"),
        QLabel("Buffer: []"),
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
    return panel, navigation


def test_navigation_is_plain_secondary_status(application):
    panel, navigation = make_panel()

    assert navigation.styleSheet() == ""
    assert panel.grid_layout.indexOf(navigation) >= 0

    panel.close()


def test_battery_uses_primary_status_box_styling(application):
    panel, _navigation = make_panel()

    assert "border" in panel.battery_label.styleSheet()
    assert "border-radius" in panel.battery_label.styleSheet()

    panel.close()
