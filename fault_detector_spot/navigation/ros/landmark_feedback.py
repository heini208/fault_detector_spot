"""ROS transport for discrete landmark recovery and refinement events."""

from dataclasses import dataclass
import math
import time
from uuid import uuid4

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


LANDMARK_FEEDBACK_TOPIC = "fault_detector/navigation_landmark_feedback"
LANDMARK_FEEDBACK_NAME = "landmark_navigation"
LANDMARK_EVENTS = frozenset({"requested", "confirmed", "failed", "refined"})


@dataclass(frozen=True)
class LandmarkFeedbackEvent:
    event_id: str
    event: str
    map_id: str
    tag_id: int
    recorded_at: float
    detail: str

    def validate(self):
        if self.event not in LANDMARK_EVENTS:
            raise ValueError("Unknown landmark event")
        if any(not isinstance(value, str) or not value.strip()
               for value in (self.event_id, self.map_id, self.detail)):
            raise ValueError("Landmark feedback requires event ID, map and detail")
        if type(self.tag_id) is not int or self.tag_id < 0:
            raise ValueError("Landmark feedback requires a nonnegative tag ID")
        if (type(self.recorded_at) not in (int, float)
                or not math.isfinite(self.recorded_at) or self.recorded_at <= 0):
            raise ValueError("Landmark event time must be positive finite UTC seconds")

    def to_message(self):
        self.validate()
        return DiagnosticStatus(
            name=LANDMARK_FEEDBACK_NAME,
            level=DiagnosticStatus.WARN if self.event == "failed" else DiagnosticStatus.OK,
            message=self.detail,
            values=[KeyValue(key=key, value=str(value)) for key, value in (
                ("event_id", self.event_id), ("event", self.event),
                ("map_id", self.map_id), ("tag_id", self.tag_id),
                ("recorded_at", self.recorded_at),
            )],
        )

    @classmethod
    def from_message(cls, message):
        if message.name != LANDMARK_FEEDBACK_NAME:
            raise ValueError("Unexpected landmark feedback source")
        values = {item.key: item.value for item in message.values}
        try:
            event = cls(
                event_id=values["event_id"], event=values["event"],
                map_id=values["map_id"], tag_id=int(values["tag_id"]),
                recorded_at=float(values["recorded_at"]), detail=message.message,
            )
        except (KeyError, TypeError, ValueError) as exception:
            raise ValueError("Malformed landmark feedback") from exception
        event.validate()
        return event


class LandmarkFeedbackPublisher:
    """Retain the latest historical event; the supervisor owns event timing."""

    def __init__(self, node, *, wall_clock=time.time):
        self.node = node
        self._wall_clock = wall_clock
        self._failure_reported = False
        self._publisher = node.create_publisher(
            DiagnosticStatus, LANDMARK_FEEDBACK_TOPIC, LATCHED_QOS,
        )

    def publish(self, event, map_id, tag_id, detail):
        feedback = LandmarkFeedbackEvent(
            uuid4().hex, event, map_id, tag_id, self._wall_clock(), detail,
        )
        message = feedback.to_message()
        if self._publisher is None:
            return False
        try:
            self._publisher.publish(message)
        except RuntimeError as exception:
            # RCLError and InvalidHandle are RuntimeErrors. Diagnostics must
            # not change the outcome of an already-issued seed or saved edit.
            if not self._failure_reported:
                self._failure_reported = True
                self.node.get_logger().warning(f"Could not publish landmark feedback: {exception}")
            return False
        self._failure_reported = False
        return True

    def close(self):
        if self._publisher is not None:
            self.node.destroy_publisher(self._publisher)
            self._publisher = None
