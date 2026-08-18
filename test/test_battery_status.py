"""Tests for robot battery status publishing and presentation."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QWidget
from spot_msgs.msg import PowerState
from std_msgs.msg import Float32

from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.ui.shared.status_overview_panel import (
    StatusOverviewPanel,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.subscription_args = None

    def create_subscription(self, *args):
        self.subscription_args = args
        return object()


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def make_status_panel(parent=None):
    return StatusOverviewPanel(
        QLabel("Status: Connected"),
        QLabel("Command: IDLE"),
        QLabel("Navigation: OFF"),
        QLabel("Visible tags: []"),
        QLabel("Buffer: []"),
        QLabel("●"),
        QLabel("No sensor"),
        QPushButton("✓"),
        QPushButton("EMERGENCY STOP"),
        parent,
    )


def test_battery_percentage_is_published_only_when_value_changes():
    behavior = BufferStatusPublisher()
    behavior.battery_percentage_pub = FakePublisher()

    power_state = PowerState()
    power_state.locomotion_charge_percentage = 63.4
    behavior._process_power_state(power_state)

    behavior._publish_battery_percentage_if_changed()
    behavior._publish_battery_percentage_if_changed()

    messages = behavior.battery_percentage_pub.messages
    assert len(messages) == 1
    assert messages[0].data == pytest.approx(63.4)


def test_battery_percentage_is_clamped_to_valid_display_range():
    behavior = BufferStatusPublisher()
    power_state = PowerState()
    power_state.locomotion_charge_percentage = 105.0

    behavior._process_power_state(power_state)

    assert behavior.latest_battery_percentage == 100.0


def test_status_panel_subscribes_and_places_battery_after_sensor(application):
    parent = QWidget()
    parent.node = FakeNode()
    sensor_status = QLabel("Probe")
    confirm = QPushButton("✓")
    panel = StatusOverviewPanel(
        QLabel("Status: Connected"),
        QLabel("Command: IDLE"),
        QLabel("Navigation: OFF"),
        QLabel("Visible tags: []"),
        QLabel("Buffer: []"),
        QLabel("●"),
        sensor_status,
        confirm,
        QPushButton("EMERGENCY STOP"),
        parent,
    )

    subscription_args = parent.node.subscription_args
    assert subscription_args[0] is Float32
    assert subscription_args[1] == "fault_detector/state/battery_percentage"

    sensor_layout = panel.sensor_widget.layout()
    assert sensor_layout.indexOf(panel.battery_label) > sensor_layout.indexOf(
        confirm
    )

    message = Float32()
    message.data = 63.6
    panel._process_battery_percentage(message)
    assert panel.battery_label.text() == "Battery: 64%"
