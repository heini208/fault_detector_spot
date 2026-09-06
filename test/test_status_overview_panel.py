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
    agent_indicator = QLabel("●")
    agent_endpoint = QPushButton("192.0.2.10:8888")
    agent_copy = QPushButton("Copy")
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
        agent_indicator,
        agent_endpoint,
        agent_copy,
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
    assert layout.getItemPosition(layout.indexOf(panel.hardware_widget)) == (
        2,
        0,
        1,
        3,
    )
    hardware_layout = panel.hardware_widget.layout()
    assert hardware_layout.indexOf(panel.agent_widget) >= 0
    assert hardware_layout.indexOf(panel.sensor_widget) >= 0
    assert hardware_layout.indexOf(panel.agent_widget) < (
        hardware_layout.indexOf(panel.sensor_widget)
    )
    assert panel.sensor_widget.layout().indexOf(sensor_indicator) >= 0
    assert panel.sensor_widget.layout().indexOf(sensor) >= 0
    assert panel.sensor_widget.layout().indexOf(confirm) >= 0
    assert panel.agent_widget.layout().indexOf(agent_indicator) >= 0
    assert panel.agent_widget.layout().indexOf(agent_endpoint) >= 0
    assert panel.agent_widget.layout().indexOf(agent_copy) >= 0

    assert buffer_label.sizePolicy().horizontalPolicy() == (
        QSizePolicy.Expanding
    )
    assert visible.sizePolicy().horizontalPolicy() == QSizePolicy.Expanding
    assert sensor.sizePolicy().horizontalPolicy() == QSizePolicy.Maximum
    sensor_layout = panel.sensor_widget.layout()
    assert sensor_layout.indexOf(sensor) + 1 == sensor_layout.indexOf(confirm)
    assert hardware_layout.itemAt(
        hardware_layout.count() - 1
    ).spacerItem() is not None
