"""Discover micro-ROS sensor heads and match them to attachment state."""

import re
import time
from typing import Dict, Iterable, Optional, Sequence, Tuple

from fault_detector_msgs.msg import (
    MicroRosAgentState,
    SensorAttachmentState,
    SensorHeadConnectionState,
)

from fault_detector_spot.shared.ros.qos_profiles import (
    APPLICATION_STATE_QOS,
    LATCHED_QOS,
)

import rclpy
from rclpy.node import Node


STATUS_TOPIC = "fault_detector/state/sensor_head_connection"
AGENT_STATUS_TOPIC = "fault_detector/state/micro_ros_agent"
ATTACHMENT_TOPIC = "fault_detector/application/sensor_attachment_state"
ACQUISITION_SERVICE_TYPE = (
    "fault_detector_msgs/srv/SetSensorAcquisition"
)
_ACQUISITION_SERVICE_PATTERN = re.compile(
    r"^/fault_detector/sensors/([a-z][a-z0-9_]*)/set_acquisition$"
)


def discovered_sensor_ids(
    services: Iterable[Tuple[str, Sequence[str]]],
) -> Tuple[str, ...]:
    """Extract valid sensor IDs from exact acquisition service endpoints."""
    sensor_ids = set()
    for service_name, service_types in services:
        if ACQUISITION_SERVICE_TYPE not in service_types:
            continue
        normalized_name = f"/{service_name.lstrip('/')}"
        match = _ACQUISITION_SERVICE_PATTERN.fullmatch(normalized_name)
        if match is not None:
            sensor_ids.add(match.group(1))
    return tuple(sorted(sensor_ids))


class SensorHeadPresenceTracker:
    """Keep brief ROS graph dropouts from flickering connection state."""

    def __init__(self, absence_grace_sec: float = 3.0):
        """Configure how long a missing graph endpoint remains present."""
        self.absence_grace_sec = max(0.0, float(absence_grace_sec))
        self._last_seen: Dict[str, float] = {}

    def observe(
        self,
        sensor_ids: Iterable[str],
        now: Optional[float] = None,
    ) -> Tuple[str, ...]:
        """Record one graph observation and return grace-filtered IDs."""
        observed_at = time.monotonic() if now is None else float(now)
        for sensor_id in sensor_ids:
            self._last_seen[sensor_id] = observed_at
        expired = []
        for sensor_id, last_seen_at in self._last_seen.items():
            age = observed_at - last_seen_at
            if age > self.absence_grace_sec:
                expired.append(sensor_id)
        for sensor_id in expired:
            del self._last_seen[sensor_id]
        return tuple(sorted(self._last_seen))


def selected_sensor_id(attachment: SensorAttachmentState) -> str:
    """Return the sensor identity relevant to the current selection."""
    pending = SensorAttachmentState.STATUS_CONFIRMATION_PENDING
    if attachment.status == pending:
        return attachment.pending_sensor_id.strip()
    return attachment.active_sensor_id.strip()


def classify_connection(
    agent_running: Optional[bool],
    attachment: Optional[SensorAttachmentState],
    connected_sensor_ids: Sequence[str],
) -> Tuple[int, str, str]:
    """Return public state, expected ID, and diagnostic detail."""
    connected = tuple(sorted(set(connected_sensor_ids)))
    if agent_running is None:
        return (
            SensorHeadConnectionState.STATE_UNKNOWN,
            "",
            "micro-ROS Agent status is unavailable",
        )
    if not agent_running:
        return (
            SensorHeadConnectionState.STATE_AGENT_UNAVAILABLE,
            "",
            "micro-ROS Agent is unavailable",
        )
    if attachment is None:
        return (
            SensorHeadConnectionState.STATE_UNKNOWN,
            "",
            "sensor attachment state is unavailable",
        )

    expected = selected_sensor_id(attachment)
    if not connected:
        detail = "No sensor heads are connected to the Agent"
        if expected:
            detail = f"Expected sensor head '{expected}' is offline"
        return SensorHeadConnectionState.STATE_NO_HEADS, expected, detail
    if not expected:
        names = ", ".join(connected)
        return (
            SensorHeadConnectionState.STATE_UNASSIGNED,
            "",
            f"Connected sensor head is not assigned: {names}",
        )
    if expected in connected:
        detail = f"Connected sensor head matches '{expected}'"
        additional = tuple(
            sensor_id for sensor_id in connected if sensor_id != expected
        )
        if additional:
            detail += "; additional heads: " + ", ".join(additional)
        return SensorHeadConnectionState.STATE_MATCHED, expected, detail
    return (
        SensorHeadConnectionState.STATE_MISMATCH,
        expected,
        f"Expected '{expected}', connected: {', '.join(connected)}",
    )


class SensorHeadConnectionNode(Node):
    """Publish derived sensor-head reachability and identity matching."""

    def __init__(self):
        """Create graph polling, attachment, Agent, and state endpoints."""
        super().__init__("sensor_head_connection")
        self.declare_parameter("sensor_head.poll_period_sec", 1.0)
        self.declare_parameter("sensor_head.absence_grace_sec", 3.0)
        self.declare_parameter("sensor_head.agent_stale_after_sec", 3.0)

        poll_period = max(
            0.2,
            float(
                self.get_parameter("sensor_head.poll_period_sec").value
            ),
        )
        absence_grace = float(
            self.get_parameter("sensor_head.absence_grace_sec").value
        )
        self.agent_stale_after_sec = max(
            0.2,
            float(
                self.get_parameter(
                    "sensor_head.agent_stale_after_sec"
                ).value
            ),
        )

        self._presence = SensorHeadPresenceTracker(absence_grace)
        self._attachment = None
        self._agent_running = None
        self._agent_received_at = None
        self._last_log_signature = None
        self.publisher = self.create_publisher(
            SensorHeadConnectionState,
            STATUS_TOPIC,
            LATCHED_QOS,
        )
        self._attachment_subscription = self.create_subscription(
            SensorAttachmentState,
            ATTACHMENT_TOPIC,
            self._receive_attachment,
            APPLICATION_STATE_QOS,
        )
        self._agent_subscription = self.create_subscription(
            MicroRosAgentState,
            AGENT_STATUS_TOPIC,
            self._receive_agent,
            LATCHED_QOS,
        )
        self.timer = self.create_timer(poll_period, self.poll)
        self.poll()

    def _receive_attachment(self, message: SensorAttachmentState) -> None:
        self._attachment = message
        self.poll()

    def _receive_agent(self, message: MicroRosAgentState) -> None:
        self._agent_running = bool(message.running)
        self._agent_received_at = time.monotonic()
        self.poll()

    def _fresh_agent_state(self, now: float) -> Optional[bool]:
        if self._agent_received_at is None:
            return None
        if now - self._agent_received_at > self.agent_stale_after_sec:
            return None
        return self._agent_running

    def poll(self) -> SensorHeadConnectionState:
        """Inspect the graph and publish one connection snapshot."""
        now = time.monotonic()
        observed = discovered_sensor_ids(
            self.get_service_names_and_types()
        )
        connected = self._presence.observe(observed, now)
        agent_running = self._fresh_agent_state(now)
        if agent_running is False:
            connected = ()
        state, expected, detail = classify_connection(
            agent_running,
            self._attachment,
            connected,
        )

        message = SensorHeadConnectionState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = state
        message.expected_sensor_id = expected
        message.connected_sensor_ids = list(connected)
        message.detail = detail
        self.publisher.publish(message)

        signature = (state, expected, connected, detail)
        if signature != self._last_log_signature:
            self._last_log_signature = signature
            self.get_logger().info(detail)
        return message


def main(args=None):
    """Run the sensor-head connection authority."""
    rclpy.init(args=args)
    node = SensorHeadConnectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
