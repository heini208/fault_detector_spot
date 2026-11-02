"""Expose live ROS topic/type pairs as optional sensor-channel suggestions."""

import time

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_spot.ui.sensor.models import SensorTopicSuggestion


_INTERNAL_TOPICS = {"/parameter_events", "/rosout"}


def normalize_topic_suggestions(topics_and_types):
    """Return stable, usable topic/type suggestions from the ROS graph."""
    suggestions = []
    for topic, message_types in topics_and_types:
        normalized_topic = f"/{str(topic).lstrip('/')}"
        normalized_types = tuple(
            sorted(
                {
                    str(message_type).strip()
                    for message_type in message_types
                    if str(message_type).strip()
                }
            )
        )
        if (
            normalized_topic in _INTERNAL_TOPICS
            or not normalized_types
        ):
            continue
        suggestions.append(
            SensorTopicSuggestion(
                topic=normalized_topic,
                message_types=normalized_types,
            )
        )
    return tuple(
        sorted(
            suggestions,
            key=lambda value: (
                not value.topic.startswith("/sensors/"),
                value.topic,
            ),
        )
    )


class SensorTopicSuggestionClient(QObject):
    """Poll the local ROS graph without making it UI presentation state."""

    suggestions_changed = pyqtSignal(object)
    discovery_error_changed = pyqtSignal(str)

    def __init__(self, node, poll_period_sec=2.0, *, monotonic_clock=time.monotonic):
        super().__init__()
        self.node = node
        self._last_suggestions = None
        self.discovery_error = ""
        self._clock = monotonic_clock
        self._last_warning_at = float("-inf")
        self._timer = node.create_timer(
            max(0.5, float(poll_period_sec)),
            self.poll,
        )

    def poll(self):
        """Publish changed topic/type suggestions to the Qt boundary."""
        try:
            suggestions = normalize_topic_suggestions(
                self.node.get_topic_names_and_types()
            )
        except Exception as error:
            message = str(error) or type(error).__name__
            now = self._clock()
            if now - self._last_warning_at >= 30.0:
                self._last_warning_at = now
                self.node.get_logger().warning(f"Sensor topic discovery failed: {message}")
            if message != self.discovery_error:
                self.discovery_error = message
                self.discovery_error_changed.emit(message)
            return self._last_suggestions or ()
        if self.discovery_error:
            self.discovery_error = ""
            self.discovery_error_changed.emit("")
            self.node.get_logger().info("Sensor topic discovery recovered")
        if suggestions != self._last_suggestions:
            self._last_suggestions = suggestions
            self.suggestions_changed.emit(suggestions)
        return suggestions

    def destroy(self):
        """Destroy the graph polling timer."""
        self.node.destroy_timer(self._timer)


__all__ = [
    "SensorTopicSuggestionClient",
    "normalize_topic_suggestions",
]
