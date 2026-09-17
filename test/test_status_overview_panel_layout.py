"""Focused layout checks for the global status overview."""

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
    recording_indicator = QLabel("●")
    recording_button = QPushButton("Record")
    panel = StatusOverviewPanel(
        QLabel("Status"),
        QLabel("Command: IDLE"),
        QLabel("Navigation: OFF"),
        QLabel("Visible tags: []"),
        QLabel("Buffer: []"),
        QLabel("●"),
        QLabel("hand_closed"),
        QPushButton("✓"),
        recording_indicator,
        recording_button,
        QPushButton("Folder"),
        QLabel("●"),
        QLabel("Offline"),
        QLabel("●"),
        QPushButton("Show IP"),
        QPushButton("Copy"),
        QPushButton("EMERGENCY STOP"),
    )
    return panel, recording_indicator, recording_button


def test_sensor_system_is_grouped_into_one_lower_strip(application):
    panel, recording_indicator, _recording_button = make_panel()

    assert panel.sensor_strip is panel.sensor_widget
    assert panel.sensor_strip is panel.hardware_widget
    assert recording_indicator.isHidden()

    panel.close()


def test_record_button_itself_shows_active_recording_state(application):
    panel, _recording_indicator, recording_button = make_panel()

    recording_button.setText("Stop")
    panel._refresh_recording_button_style()

    assert "#C62828" in recording_button.styleSheet()

    recording_button.setText("Record")
    panel._refresh_recording_button_style()

    assert "#C62828" not in recording_button.styleSheet()

    panel.close()
