"""Tests for the sensor-head connection Qt adapter and status rendering."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication, QLabel, QWidget  # noqa: E402, I100

from fault_detector_msgs.msg import (  # noqa: E402, I100
    SensorHeadConnectionState,
)

import pytest  # noqa: E402, I100

from fault_detector_spot.ui.fault_detector_ui import (  # noqa: E402, I100
    Fault_Detector_UI,
)
from fault_detector_spot.ui.ros.sensor_head_connection_client import (  # noqa: E402, E501, I100
    SensorHeadConnectionClient,
)


class _Node:

    def __init__(self):
        self.callback = None
        self.destroyed = None

    def create_subscription(self, _type, _topic, callback, _qos):
        self.callback = callback
        return object()

    def destroy_subscription(self, subscription):
        self.destroyed = subscription


class _ConnectionControls(QWidget):

    set_sensor_connection_status = (
        Fault_Detector_UI.set_sensor_connection_status
    )
    refresh = Fault_Detector_UI._refresh_sensor_connection_status

    def __init__(self):
        super().__init__()
        self.sensor_connection_indicator_label = QLabel("●", self)
        self.sensor_connection_status_label = QLabel("Unknown", self)
        self._sensor_head_connection_state = None


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application shared by widget tests."""
    return QApplication.instance() or QApplication([])


def test_client_converts_message_and_tracks_freshness(application):
    """Convert ROS constants into stable presentation state."""
    node = _Node()
    client = SensorHeadConnectionClient(node)
    message = SensorHeadConnectionState()
    message.state = SensorHeadConnectionState.STATE_MATCHED
    message.expected_sensor_id = "bmm150_probe"
    message.connected_sensor_ids = ["bmm150_probe"]
    message.detail = "Connected sensor head matches"
    received = []
    client.state_changed.connect(received.append)

    node.callback(message)

    assert received[0].status.value == "matched"
    assert received[0].connected_sensor_ids == ("bmm150_probe",)
    assert not client.is_stale(now=client.last_received_at + 2.9)
    assert client.is_stale(now=client.last_received_at + 3.1)
    client.destroy()
    assert node.destroyed is not None


def test_main_status_renders_connected_and_mismatch(application):
    """Use separate visual states for a match and an ID mismatch."""
    view = _ConnectionControls()
    node = _Node()
    client = SensorHeadConnectionClient(node)

    matched = SensorHeadConnectionState()
    matched.state = SensorHeadConnectionState.STATE_MATCHED
    matched.expected_sensor_id = "bmm150_probe"
    matched.connected_sensor_ids = ["bmm150_probe"]
    matched.detail = "match"
    view._sensor_head_connection_state = client._state_view(matched)
    view.refresh()

    assert view.sensor_connection_status_label.text() == "Connected"
    assert "#2E7D32" in view.sensor_connection_indicator_label.styleSheet()

    mismatch = SensorHeadConnectionState()
    mismatch.state = SensorHeadConnectionState.STATE_MISMATCH
    mismatch.expected_sensor_id = "thermal_probe"
    mismatch.connected_sensor_ids = ["bmm150_probe"]
    mismatch.detail = "Expected thermal_probe"
    view._sensor_head_connection_state = client._state_view(mismatch)
    view.refresh()

    assert view.sensor_connection_status_label.text() == "ID mismatch"
    assert "#C62828" in view.sensor_connection_indicator_label.styleSheet()
