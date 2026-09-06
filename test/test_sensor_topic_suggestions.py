"""Tests for optional ROS graph suggestions in sensor mount setup."""

from types import SimpleNamespace

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

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed_timers.append(timer)

    def get_topic_names_and_types(self):
        return self.topics


def test_topic_suggestions_are_sorted_and_filter_ros_plumbing():
    suggestions = normalize_topic_suggestions(
        (
            ("/rosout", ("rcl_interfaces/msg/Log",)),
            ("sensors/z", ("std_msgs/msg/Float64",)),
            ("/sensors/a", ("std_msgs/msg/String",)),
        )
    )

    assert [value.topic for value in suggestions] == [
        "/sensors/a",
        "/sensors/z",
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
