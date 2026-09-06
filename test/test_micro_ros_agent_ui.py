"""Tests for the micro-ROS Agent UI status adapter and controls."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402, I100
from PyQt5.QtWidgets import (  # noqa: E402, I100
    QApplication,
    QLabel,
    QPushButton,
    QWidget,
)
from fault_detector_msgs.msg import MicroRosAgentState  # noqa: E402, I100

from fault_detector_spot.ui.fault_detector_ui import (  # noqa: E402, I100
    Fault_Detector_UI,
)
from fault_detector_spot.ui.ros.micro_ros_agent_status_client import (  # noqa: E402, E501, I100
    MicroRosAgentStatusClient,
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


class _AgentControls(QWidget):

    set_status = Fault_Detector_UI.set_micro_ros_agent_status
    _set_agent_endpoint_visibility = (
        Fault_Detector_UI._set_agent_endpoint_visibility
    )
    _refresh_agent_endpoint_visibility = (
        Fault_Detector_UI._refresh_agent_endpoint_visibility
    )
    _copy_agent_configuration = (
        Fault_Detector_UI._copy_agent_configuration
    )
    _restore_agent_copy_button_text = (
        Fault_Detector_UI._restore_agent_copy_button_text
    )

    def __init__(self):
        super().__init__()
        self.agent_indicator_label = QLabel("●", self)
        self.agent_endpoint_button = QPushButton("Show IP", self)
        self.agent_endpoint_button.setCheckable(True)
        self.agent_copy_button = QPushButton("Copy", self)
        self.agent_copy_button.setVisible(False)
        self._agent_endpoint_text = "IP unavailable"
        self._agent_command = ""
        self._agent_endpoint_visible = False


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_client_tracks_status_freshness(application):
    node = _Node()
    client = MicroRosAgentStatusClient(node, stale_after_sec=3.0)
    message = MicroRosAgentState()

    node.callback(message)

    assert not client.is_stale(now=client.last_received_at + 2.9)
    assert client.is_stale(now=client.last_received_at + 3.1)
    client.destroy()
    assert node.destroyed is not None


def test_endpoint_is_hidden_until_revealed_and_command_can_be_copied(
    application,
):
    view = _AgentControls()

    view.set_status(
        True,
        "192.168.178.69",
        8888,
        "Agent running",
    )

    assert view.agent_endpoint_button.isEnabled()
    assert view._agent_endpoint_text == "192.168.178.69:8888"
    assert view.agent_endpoint_button.text() == "Show IP"
    assert view.agent_copy_button.isHidden()

    view._set_agent_endpoint_visibility(True)
    assert view.agent_endpoint_button.text() == "192.168.178.69:8888"
    assert not view.agent_copy_button.isHidden()

    view._copy_agent_configuration()
    assert QApplication.clipboard().text() == (
        "set-agent 192.168.178.69 8888"
    )

    view._set_agent_endpoint_visibility(False)
    assert view.agent_endpoint_button.text() == "Show IP"
    assert view.agent_copy_button.isHidden()
