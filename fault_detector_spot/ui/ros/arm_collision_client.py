"""Qt transport for the authoritative arm collision setting."""

from dataclasses import dataclass
import time

from diagnostic_msgs.msg import DiagnosticStatus
from PyQt5.QtCore import QObject, pyqtSignal
from std_srvs.srv import SetBool

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


@dataclass(frozen=True)
class ArmCollisionState:
    available: bool
    enabled: bool


class ArmCollisionClient(QObject):
    state_changed = pyqtSignal(object)
    request_finished = pyqtSignal(bool, str)
    REQUEST_TIMEOUT_SEC = 3.0

    def __init__(self, node, stale_after_sec=3.0, monotonic_clock=time.monotonic):
        super().__init__()
        self.node = node
        self.stale_after_sec = float(stale_after_sec)
        self._clock = monotonic_clock
        self.last_state = None
        self.last_received_at = None
        self._pending = None
        self._request_started_at = None
        self._destroyed = False
        self._subscription = node.create_subscription(
            DiagnosticStatus, "fault_detector/arm_collision_checking",
            self._receive_state, LATCHED_QOS,
        )
        self._client = node.create_client(
            SetBool, "fault_detector/set_arm_collision_checking",
        )

    @property
    def pending(self):
        return self._pending is not None

    @property
    def pending_timed_out(self):
        started_at = self._request_started_at
        return (self.pending and started_at is not None
                and self._clock() - started_at > self.REQUEST_TIMEOUT_SEC)

    def is_stale(self):
        return (self.last_received_at is None or
                self._clock() - self.last_received_at > self.stale_after_sec)

    def _receive_state(self, message):
        if self._destroyed:
            return
        values = {value.key: value.value for value in message.values}
        available = values.get("arm_collision_available")
        enabled = values.get("arm_collision_enabled")
        if available not in ("true", "false") or enabled not in ("true", "false"):
            return
        self.last_state = ArmCollisionState(
            available=available == "true",
            enabled=available == "true" and enabled == "true",
        )
        self.last_received_at = self._clock()
        self.state_changed.emit(self.last_state)

    def set_enabled(self, enabled):
        """Request a setting change; diagnostics remain the displayed truth."""
        if self._destroyed or self.pending:
            return None
        if not self._client.service_is_ready():
            self.request_finished.emit(False, "Arm collision control service is unavailable")
            return None
        try:
            future = self._client.call_async(SetBool.Request(data=enabled))
        except Exception as exception:
            self.request_finished.emit(False, str(exception))
            return None
        self._request_started_at = self._clock()
        self._pending = future
        future.add_done_callback(self._receive_result)
        return future

    def _receive_result(self, future):
        if self._destroyed or future is not self._pending:
            return
        try:
            response = future.result()
            if response is None:
                raise RuntimeError("Arm collision control returned no response")
            success, detail = response.success, response.message
        except Exception as exception:
            success, detail = False, str(exception)
        # Release admission only after finishing the old request's state changes.
        self._request_started_at = None
        self._pending = None
        self.request_finished.emit(success, detail)

    def destroy(self):
        self._destroyed = True
        self.node.destroy_subscription(self._subscription)
        self.node.destroy_client(self._client)
