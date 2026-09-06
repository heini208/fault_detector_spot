"""Qt adapter for sensor-head connection and identity state."""

import time

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_msgs.msg import SensorHeadConnectionState

from fault_detector_spot.sensing.sensor_head_connection_node import (
    STATUS_TOPIC,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.ui.sensor.models import (
    SensorHeadConnectionView,
    SensorHeadConnectionViewStatus,
)


class SensorHeadConnectionClient(QObject):
    """Expose typed sensor-head connection snapshots to the UI."""

    state_changed = pyqtSignal(object)

    def __init__(self, node, stale_after_sec: float = 3.0):
        """Subscribe and configure local freshness tracking."""
        super().__init__()
        self.node = node
        self.stale_after_sec = float(stale_after_sec)
        self.last_state = None
        self.last_received_at = None
        self._subscription = node.create_subscription(
            SensorHeadConnectionState,
            STATUS_TOPIC,
            self._receive_state,
            LATCHED_QOS,
        )

    def _receive_state(self, message) -> None:
        self.last_state = self._state_view(message)
        self.last_received_at = time.monotonic()
        self.state_changed.emit(self.last_state)

    def is_stale(self, now: float = None) -> bool:
        """Return whether no recent connection snapshot was received."""
        if self.last_received_at is None:
            return True
        current = time.monotonic() if now is None else float(now)
        return current - self.last_received_at > self.stale_after_sec

    @staticmethod
    def _state_view(message) -> SensorHeadConnectionView:
        values = {
            SensorHeadConnectionState.STATE_AGENT_UNAVAILABLE: (
                SensorHeadConnectionViewStatus.AGENT_UNAVAILABLE
            ),
            SensorHeadConnectionState.STATE_NO_HEADS: (
                SensorHeadConnectionViewStatus.NO_HEADS
            ),
            SensorHeadConnectionState.STATE_UNASSIGNED: (
                SensorHeadConnectionViewStatus.UNASSIGNED
            ),
            SensorHeadConnectionState.STATE_MATCHED: (
                SensorHeadConnectionViewStatus.MATCHED
            ),
            SensorHeadConnectionState.STATE_MISMATCH: (
                SensorHeadConnectionViewStatus.MISMATCH
            ),
        }
        return SensorHeadConnectionView(
            status=values.get(
                int(message.state),
                SensorHeadConnectionViewStatus.UNKNOWN,
            ),
            expected_sensor_id=message.expected_sensor_id.strip(),
            connected_sensor_ids=tuple(message.connected_sensor_ids),
            detail=message.detail.strip(),
        )

    def destroy(self) -> None:
        """Destroy the ROS subscription owned by this adapter."""
        self.node.destroy_subscription(self._subscription)


__all__ = ["SensorHeadConnectionClient"]
