"""Qt adapter for micro-ROS Agent endpoint and health state."""

import time

from fault_detector_msgs.msg import MicroRosAgentState
from fault_detector_spot.sensing.micro_ros_agent_status_node import STATUS_TOPIC
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from PyQt5.QtCore import pyqtSignal, QObject


class MicroRosAgentStatusClient(QObject):
    """Expose Agent status messages and freshness to the UI."""

    state_changed = pyqtSignal(object)

    def __init__(self, node, stale_after_sec: float = 3.0):
        super().__init__()
        self.node = node
        self.stale_after_sec = float(stale_after_sec)
        self.last_state = None
        self.last_received_at = None
        self._subscription = node.create_subscription(
            MicroRosAgentState,
            STATUS_TOPIC,
            self._receive_state,
            LATCHED_QOS,
        )

    def _receive_state(self, message) -> None:
        self.last_state = message
        self.last_received_at = time.monotonic()
        self.state_changed.emit(message)

    def is_stale(self, now: float = None) -> bool:
        if self.last_received_at is None:
            return True
        current = time.monotonic() if now is None else float(now)
        return current - self.last_received_at > self.stale_after_sec

    def destroy(self) -> None:
        self.node.destroy_subscription(self._subscription)


__all__ = ["MicroRosAgentStatusClient"]
