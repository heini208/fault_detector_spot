"""Provide the PyQt UI with the typed remote application API."""

from copy import deepcopy
from functools import partial
from uuid import uuid4

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
)
from fault_detector_msgs.srv import CancelOperation, EmergencyStop
from rclpy.action import ActionClient

from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


class ApplicationClient(QObject):
    """Submit operational intents and expose authoritative state signals."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)
    emergency_stop_finished = pyqtSignal(bool, str)

    def __init__(self, node, client_id=None):
        super().__init__()
        self.node = node
        self.client_id = self._client_id(client_id)
        self._last_state_fingerprint = None
        self._goal_handles = {}
        self._operation_client = ActionClient(
            node,
            ExecuteOperation,
            "fault_detector/application/execute_operation",
        )
        self._cancel_client = node.create_client(
            CancelOperation,
            "fault_detector/application/cancel_operation",
        )
        self._emergency_client = node.create_client(
            EmergencyStop,
            "fault_detector/application/emergency_stop",
        )
        self._state_subscription = node.create_subscription(
            ApplicationCommandState,
            "fault_detector/application/command_state",
            self._receive_state,
            APPLICATION_STATE_QOS,
        )

    def execute(
        self,
        intent: OperationalIntent,
        context_id: str = "",
    ):
        """Submit one typed operational intent without blocking the UI."""
        if not isinstance(intent, OperationalIntent):
            raise TypeError("Expected an OperationalIntent message")
        if not self._operation_client.server_is_ready():
            self.request_rejected.emit(
                "Application operation server is unavailable"
            )
            return None
        goal = ExecuteOperation.Goal()
        goal.client_id = self.client_id
        goal.context_id = context_id.strip()
        goal.intent = deepcopy(intent)
        local_id = uuid4().hex
        future = self._operation_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._receive_feedback,
                local_id,
            ),
        )
        future.add_done_callback(
            partial(self._receive_goal_response, local_id)
        )
        return local_id

    def cancel(self, request_id: str):
        """Request cancellation of one correlated operation."""
        if not self._cancel_client.service_is_ready():
            self.request_rejected.emit(
                "Application cancellation service is unavailable"
            )
            return None
        request = CancelOperation.Request()
        request.client_id = self.client_id
        request.request_id = request_id.strip()
        future = self._cancel_client.call_async(request)
        future.add_done_callback(self._receive_cancellation_result)
        return future

    def emergency_stop(self):
        """Request immediate global command cancellation."""
        if not self._emergency_client.service_is_ready():
            self.emergency_stop_finished.emit(
                False,
                "Emergency-stop service is unavailable",
            )
            return None
        request = EmergencyStop.Request()
        request.client_id = self.client_id
        future = self._emergency_client.call_async(request)
        future.add_done_callback(self._receive_emergency_result)
        return future

    def _receive_goal_response(self, local_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if not goal_handle.accepted:
            self.request_rejected.emit("Operational request was rejected")
            return
        self._goal_handles[local_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._receive_result, local_id)
        )

    def _receive_feedback(self, _local_id, feedback_message):
        self._emit_state(feedback_message.feedback.state)

    def _receive_result(self, local_id, future):
        self._goal_handles.pop(local_id, None)
        try:
            result = future.result().result
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        self._emit_state(result.state)

    def _receive_state(self, state):
        self._emit_state(state)

    def _emit_state(self, state):
        fingerprint = (
            state.request_id,
            int(state.state),
            state.detail,
            int(state.buffered_command_count),
        )
        if fingerprint == self._last_state_fingerprint:
            return
        self._last_state_fingerprint = fingerprint
        self.state_changed.emit(state)

    def _receive_cancellation_result(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if not response.accepted:
            self.request_rejected.emit(response.detail)

    def _receive_emergency_result(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.emergency_stop_finished.emit(False, str(exception))
            return
        self.emergency_stop_finished.emit(
            bool(response.accepted),
            response.detail,
        )

    def destroy(self):
        """Destroy client-side ROS entities owned by the adapter."""
        self._operation_client.destroy()
        self.node.destroy_client(self._cancel_client)
        self.node.destroy_client(self._emergency_client)
        self.node.destroy_subscription(self._state_subscription)
        self._goal_handles.clear()

    @staticmethod
    def _client_id(value):
        normalized = value.strip() if isinstance(value, str) else ""
        return normalized or f"fault_detector_ui_{uuid4().hex}"
