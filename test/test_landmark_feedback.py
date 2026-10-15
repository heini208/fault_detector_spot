"""Discrete landmark events reach a persistent UI label without ROS startup."""

from dataclasses import replace
import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton

from fault_detector_spot.navigation.ros.landmark_feedback import (
    LANDMARK_FEEDBACK_TOPIC, LandmarkFeedbackEvent, LandmarkFeedbackPublisher,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.navigation.landmark_feedback import LandmarkFeedbackLabel
from fault_detector_spot.ui.ros import navigation_setup_client as client_module
from fault_detector_spot.ui.shared.status_overview_panel import StatusOverviewPanel


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


def event(kind="requested", **changes):
    base = LandmarkFeedbackEvent(
        "event-1", kind, "plant", 7, 1792073242.0,
        "Tag 7 recovery requested; waiting for accepted localization results",
    )
    return replace(base, **changes)


@pytest.mark.parametrize("kind", ["requested", "confirmed", "failed", "refined"])
def test_publisher_carries_event_identity_and_wall_timestamp(kind):
    node = Mock()
    publisher = LandmarkFeedbackPublisher(node, wall_clock=lambda: 1792073242.0)
    node.create_publisher.assert_called_once_with(
        DiagnosticStatus, LANDMARK_FEEDBACK_TOPIC, LATCHED_QOS,
    )
    publisher.publish(kind, "plant", 7, "Readable event detail")
    publisher.publish(kind, "plant", 7, "Readable event detail")
    messages = [call.args[0] for call in node.create_publisher.return_value.publish.call_args_list]
    decoded = [LandmarkFeedbackEvent.from_message(message) for message in messages]
    assert decoded[0].event_id != decoded[1].event_id
    assert decoded[0].event == kind
    assert decoded[0].map_id == "plant"
    assert decoded[0].tag_id == 7
    assert decoded[0].recorded_at == 1792073242.0
    assert decoded[0].detail == "Readable event detail"
    assert messages[0].level == (DiagnosticStatus.WARN if kind == "failed" else DiagnosticStatus.OK)
    publisher.close()
    publisher.close()
    node.destroy_publisher.assert_called_once()


def test_transport_failure_is_best_effort_and_warns_once_until_delivery_recovers():
    node = Mock()
    publisher = LandmarkFeedbackPublisher(node, wall_clock=lambda: 1792073242.0)
    transport = node.create_publisher.return_value
    transport.publish.side_effect = RuntimeError("ROS context unavailable")
    assert not publisher.publish("requested", "plant", 7, "Seed was issued")
    assert not publisher.publish("confirmed", "plant", 7, "Recovery confirmed")
    node.get_logger.return_value.warning.assert_called_once()
    transport.publish.side_effect = None
    assert publisher.publish("refined", "plant", 7, "Refinement saved")
    transport.publish.side_effect = RuntimeError("Publisher invalidated")
    assert not publisher.publish("requested", "plant", 7, "Another seed issued")
    assert node.get_logger.return_value.warning.call_count == 2
    publisher.close()
    assert not publisher.publish("failed", "plant", 7, "Recovery failed")


def test_publisher_does_not_hide_invalid_event_schema():
    node = Mock()
    publisher = LandmarkFeedbackPublisher(node)
    with pytest.raises(ValueError, match="Unknown landmark event"):
        publisher.publish("typo", "plant", 7, "Detail")
    node.create_publisher.return_value.publish.assert_not_called()
    node.get_logger.return_value.warning.assert_not_called()


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(client_module, "ActionClient", Mock())
    node = Mock()
    node.create_subscription.side_effect = lambda *args: SimpleNamespace(topic=args[1], callback=args[2])
    client = client_module.NavigationSetupClient(node, "ui")
    yield client
    if not client._destroyed:
        client.destroy()


def test_feedback_delivery_is_independent_of_setup_context_and_deduplicated(client):
    received = []
    client.landmark_feedback_received.connect(received.append)
    assert client.context_id == ""
    subscription = client._landmark_subscription
    assert subscription.topic == LANDMARK_FEEDBACK_TOPIC
    first = event().to_message()
    subscription.callback(first)
    subscription.callback(first)
    assert received == [event()]
    client.context_id = "another-context"
    later = event("confirmed", event_id="event-2", detail="Recovery confirmed")
    subscription.callback(later.to_message())
    assert received == [event(), later]


@pytest.mark.parametrize("field,value", [
    ("event", "unknown"), ("map_id", ""), ("tag_id", "-1"),
    ("tag_id", "not-an-int"), ("recorded_at", "nan"),
    ("recorded_at", "inf"), ("recorded_at", "0"), ("event_id", ""),
])
def test_malformed_feedback_does_not_replace_visible_event(client, field, value):
    received = []
    client.landmark_feedback_received.connect(received.append)
    message = event().to_message()
    for item in message.values:
        if item.key == field:
            item.value = value
    client._receive_landmark_feedback(message)
    assert received == []
    assert client._last_landmark_event_id is None
    client._receive_landmark_feedback(event().to_message())
    assert len(received) == 1


def test_missing_metadata_and_wrong_source_are_ignored(client):
    received = []
    client.landmark_feedback_received.connect(received.append)
    missing = event().to_message()
    missing.values = []
    client._receive_landmark_feedback(missing)
    wrong = event().to_message()
    wrong.name = "another_source"
    client._receive_landmark_feedback(wrong)
    assert not received


def test_destroy_releases_feedback_subscription_and_ignores_late_callbacks(client):
    received = []
    client.landmark_feedback_received.connect(received.append)
    subscription = client._landmark_subscription
    client.destroy()
    client.node.destroy_subscription.assert_any_call(subscription)
    subscription.callback(event().to_message())
    assert received == []


def test_qt_handoff_renders_distinct_requested_and_confirmed_outcomes(application, client):
    label = LandmarkFeedbackLabel()
    client.landmark_feedback_received.connect(label.show_event, Qt.QueuedConnection)
    client._receive_landmark_feedback(event().to_message())
    assert "none reported" in label.text()
    application.processEvents()
    assert "Tag 7" in label.text()
    assert "plant" in label.text()
    assert "awaiting confirmation" in label.text()
    assert "2026-" in label.text()
    client._receive_landmark_feedback(event("confirmed", event_id="event-2").to_message())
    application.processEvents()
    assert "relocalization confirmed" in label.text()
    assert "awaiting confirmation" not in label.text()
    label.close()


def test_landmark_feedback_survives_general_setup_status_updates(application):
    label = LandmarkFeedbackLabel()
    label.show_event(event("refined", detail="Adjusted anchor by 4 mm; original pose preserved"))
    ui = SimpleNamespace(
        landmark_feedback_label=label, status_label=QLabel(),
        navigation_controls=Mock(), inspection_controls=Mock(),
    )
    Fault_Detector_UI._process_navigation_setup_state(ui, SimpleNamespace(detail="Runtime mode: localization"))
    assert ui.status_label.text() == "Runtime mode: localization"
    assert "landmark position refined" in label.text()
    assert "4 mm" in label._message()
    label.close()


def test_feedback_line_is_global_and_long_details_do_not_grow_layout(application):
    label = LandmarkFeedbackLabel()
    labels = [QLabel() for _ in range(11)]
    buttons = [QPushButton() for _ in range(6)]
    panel = StatusOverviewPanel(
        labels[0], labels[1], labels[2], labels[3], labels[4], labels[5], labels[6], buttons[0],
        labels[7], buttons[1], buttons[2], labels[8], labels[9], labels[10], buttons[3], buttons[4],
        buttons[5], landmark_feedback_label=label,
    )
    assert panel.grid_layout.getItemPosition(panel.grid_layout.indexOf(label)) == (2, 0, 1, 5)
    before = label.sizeHint().height()
    detail = "A long recovery failure reason <with diagnostic data>. " * 150
    label.show_event(event("failed", detail=detail))
    assert "relocalization failed" in label.text()
    assert label.sizeHint().height() == before
    assert detail in label._message()
    assert "&lt;with diagnostic data&gt;" in label.toolTip()
    panel.close()
