"""Provide the PyQt UI with the typed probe setup API."""

from copy import deepcopy
from functools import partial
from uuid import uuid4

from PyQt5.QtCore import QObject, pyqtSignal
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from fault_detector_msgs.srv import CloseProbeSetup, ExecuteProbeSetup

from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


class ProbeSetupClient(QObject):
    """Submit probe authoring intent and expose immutable state."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)
    close_finished = pyqtSignal(bool, str)

    def __init__(self, node, client_id: str):
        super().__init__()
        self.node = node
        self.client_id = client_id
        self.context_id = ""
        self._last_state_fingerprint = None
        self._pending_request_id = ""
        self._execute_client = node.create_client(
            ExecuteProbeSetup,
            "fault_detector/application/execute_probe_setup",
        )
        self._close_client = node.create_client(
            CloseProbeSetup,
            "fault_detector/application/close_probe_setup",
        )
        self._state_subscription = node.create_subscription(
            ProbeSetupState,
            "fault_detector/application/probe_setup_state",
            self._receive_state,
            APPLICATION_STATE_QOS,
        )

    def open(self):
        """Open one server-owned probe setup context."""
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_OPEN
        return self._send(intent, "")

    def execute(self, intent: ProbeSetupIntent):
        """Submit one probe authoring intent in the current context."""
        if not self.context_id:
            self.request_rejected.emit("Probe setup context is not open")
            return None
        return self._send(intent, self.context_id)

    def close(self):
        """Close the current server-owned probe setup context."""
        if not self.context_id:
            return None
        if self._pending_request_id:
            self.close_finished.emit(
                False,
                "Probe setup transaction is still in progress",
            )
            return None
        if not self._close_client.service_is_ready():
            self.close_finished.emit(
                False,
                "Probe setup close service is unavailable",
            )
            return None
        request = CloseProbeSetup.Request()
        request.client_id = self.client_id
        request.context_id = self.context_id
        future = self._close_client.call_async(request)
        future.add_done_callback(self._receive_close)
        return future

    def _send(self, intent, context_id):
        if not isinstance(intent, ProbeSetupIntent):
            raise TypeError("Expected a ProbeSetupIntent message")
        if self._pending_request_id:
            self.request_rejected.emit(
                "A probe setup transaction is already in progress"
            )
            return None
        if not self._execute_client.service_is_ready():
            return None
        request = ExecuteProbeSetup.Request()
        request.client_id = self.client_id
        request.context_id = context_id
        request.intent = deepcopy(intent)
        local_id = uuid4().hex
        self._pending_request_id = local_id
        future = self._execute_client.call_async(request)
        future.add_done_callback(
            partial(self._receive_response, local_id)
        )
        return local_id

    def _receive_response(self, local_id, future):
        if self._pending_request_id == local_id:
            self._pending_request_id = ""
        try:
            response = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        self._emit_state(response.state)

    def _receive_state(self, state):
        self._emit_state(state)

    def _emit_state(self, state):
        if state.client_id != self.client_id:
            return
        if self.context_id and state.context_id != self.context_id:
            return
        if state.context_id:
            self.context_id = state.context_id
        fingerprint = (
            state.request_id,
            int(state.state),
            int(state.revision),
            state.detail,
        )
        if fingerprint == self._last_state_fingerprint:
            return
        self._last_state_fingerprint = fingerprint
        self.state_changed.emit(state)

    def _receive_close(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.close_finished.emit(False, str(exception))
            return
        if response.closed:
            self.context_id = ""
        self.close_finished.emit(response.closed, response.detail)

    def destroy(self):
        """Destroy client-side ROS resources owned by this adapter."""
        self.node.destroy_client(self._execute_client)
        self.node.destroy_client(self._close_client)
        self.node.destroy_subscription(self._state_subscription)


__all__ = ["ProbeSetupClient"]
