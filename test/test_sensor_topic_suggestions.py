"""Tests for optional ROS graph suggestions in sensor mount setup."""

from types import SimpleNamespace
from unittest.mock import Mock

from PyQt5.QtWidgets import QApplication

from fault_detector_spot.ui.sensor.controls import SensorControls
from fault_detector_spot.ui.ros.sensor_topic_suggestion_client import (
    SensorTopicSuggestionClient,
    normalize_topic_suggestions,
)


class FakeNode:
    """Provide graph and timer surfaces used by the suggestion adapter."""

    def __init__(self, topics=()):
        self.topics = topics
        self.timers = []
        self.destroyed_timers = []
        self.error = None
        self.logger = Mock()

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed_timers.append(timer)

    def get_topic_names_and_types(self):
        if self.error is not None:
            raise self.error
        return self.topics

    def get_logger(self):
        return self.logger


def test_topic_suggestions_are_sorted_and_filter_ros_plumbing():
    suggestions = normalize_topic_suggestions(
        (
            ("/rosout", ("rcl_interfaces/msg/Log",)),
            ("/camera/image", ("sensor_msgs/msg/Image",)),
            ("sensors/z", ("std_msgs/msg/Float64",)),
            ("/sensors/a", ("std_msgs/msg/String",)),
        )
    )

    assert [value.topic for value in suggestions] == [
        "/sensors/a",
        "/sensors/z",
        "/camera/image",
    ]
    assert suggestions[0].message_types == ("std_msgs/msg/String",)


def test_suggestion_client_emits_only_when_graph_values_change():
    node = FakeNode(
        (("/sensors/probe/value", ("std_msgs/msg/Float64",)),)
    )
    client = SensorTopicSuggestionClient(node)
    emissions = []
    client.suggestions_changed.connect(emissions.append)

    first = client.poll()
    client.poll()
    node.topics = (
        ("/sensors/probe/value", ("std_msgs/msg/Float32",)),
    )
    second = client.poll()
    client.destroy()

    assert len(emissions) == 2
    assert first[0].message_types == ("std_msgs/msg/Float64",)
    assert second[0].message_types == ("std_msgs/msg/Float32",)
    assert node.destroyed_timers == node.timers


def test_discovery_failure_retains_suggestions_and_throttles_warnings_until_recovery():
    node = FakeNode((("/sensors/value", ("std_msgs/msg/Float64",)),))
    now = [0.0]
    client = SensorTopicSuggestionClient(node, monotonic_clock=lambda: now[0])
    suggestions, errors = [], []
    client.suggestions_changed.connect(suggestions.append)
    client.discovery_error_changed.connect(errors.append)
    last = client.poll()

    node.error = RuntimeError("graph unavailable")
    assert client.poll() == last
    for now[0] in (1.0, 2.0, 29.0):
        assert client.poll() == last
    assert errors == ["graph unavailable"]
    assert suggestions == [last]
    node.logger.warning.assert_called_once()
    now[0] = 30.0
    client.poll()
    assert node.logger.warning.call_count == 2

    node.error = None
    assert client.poll() == last
    client.poll()
    assert errors == ["graph unavailable", ""]
    assert client.discovery_error == ""
    assert suggestions == [last]
    node.logger.info.assert_called_once_with("Sensor topic discovery recovered")


def test_initial_discovery_failure_is_distinct_from_an_empty_graph():
    node = FakeNode()
    node.error = RuntimeError("graph unavailable")
    client = SensorTopicSuggestionClient(node)
    suggestions = []
    client.suggestions_changed.connect(suggestions.append)

    assert client.poll() == ()
    assert suggestions == []
    assert client.discovery_error == "graph unavailable"
    node.error = None
    assert client.poll() == ()
    assert suggestions == [()]
    assert client.discovery_error == ""


def test_discovery_warning_preserves_manual_channel_input_and_last_options():
    application = QApplication.instance() or QApplication([])
    controls = SensorControls()
    node = FakeNode((("/sensors/value", ("std_msgs/msg/Float64",)),))
    client = SensorTopicSuggestionClient(node)
    client.suggestions_changed.connect(controls.apply_topic_suggestions)
    client.discovery_error_changed.connect(controls.apply_topic_discovery_error)
    client.poll()
    controls.channel_topic_field.setEditText("/offline/manual")
    controls.channel_message_type_field.setEditText("custom_msgs/msg/Reading")

    node.error = RuntimeError("graph unavailable")
    client.poll()

    assert not controls.topic_discovery_warning.isHidden()
    assert controls.topic_discovery_warning.toolTip() == "graph unavailable"
    assert controls.channel_topic_field.currentText() == "/offline/manual"
    assert controls.channel_message_type_field.currentText() == "custom_msgs/msg/Reading"
    assert controls.channel_topic_field.findText("/sensors/value") >= 0
    node.error = None
    client.poll()
    assert controls.topic_discovery_warning.isHidden()
    assert controls.channel_topic_field.currentText() == "/offline/manual"
    assert application is not None
    controls.close()
