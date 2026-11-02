"""Provide the PyQt UI with the typed remote application API."""

from copy import deepcopy
from dataclasses import dataclass
from functools import partial
from threading import RLock
import time
from uuid import uuid4

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    SoftwareStopState,
)
from fault_detector_msgs.srv import CancelOperation, EmergencyStop, ResetSoftwareStop
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType

from fault_detector_spot.application.commanding.command_lifecycle import CommandLifecycleConfig
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS, LATCHED_QOS
from fault_detector_spot.ui.ros.command_state_order import CommandStateOrder


@dataclass
class _PendingOperation:
    context_id: str
    last_seen: float
    request_id: str = ""
    uncertain: bool = False
    goal_handle: object = None


class ApplicationClient(QObject):
    """Submit operational intents and expose authoritative state signals."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)
    operation_rejected = pyqtSignal(str, str)
    operation_uncertain = pyqtSignal(str, str)
    emergency_stop_finished = pyqtSignal(bool, str)
    software_stop_state_changed = pyqtSignal(object)
    software_stop_reset_finished = pyqtSignal(bool, str)

    def __init__(self, node, client_id=None, *, lifecycle=None, monotonic_clock=time.monotonic):
        super().__init__()
        self.node = node
        self._lock = RLock()
        self.client_id = self._client_id(client_id)
        self._last_state_fingerprint = None
        self._pending_operations = {}
        self._observed_states = {}
        self._state_order = CommandStateOrder()
        self._software_stop_state = None
        self._software_stop_sequence = 0
        self._software_stop_requested_locally = False
        self._software_stop_observed_latched = False
        self._software_stop_request_id = ""
        self._software_stop_generation = 0
        self._lifecycle = lifecycle or CommandLifecycleConfig()
        self._monotonic_clock = monotonic_clock
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
        self._reset_software_stop_client = node.create_client(
            ResetSoftwareStop,
            "fault_detector/application/reset_software_stop",
        )
        self._state_subscription = node.create_subscription(
            ApplicationCommandState,
            "fault_detector/application/command_state",
            self._receive_state,
            APPLICATION_STATE_QOS,
        )
        self._software_stop_subscription = node.create_subscription(
            SoftwareStopState,
            "fault_detector/application/software_stop_state",
            self._receive_software_stop_state,
            LATCHED_QOS,
        )
        self._liveness_timer = node.create_timer(
            self._lifecycle.poll_period_sec, self._poll_liveness,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def execute(
        self,
        intent: OperationalIntent,
        context_id: str = "",
    ):
        """Submit one typed operational intent without blocking the UI."""
        if not isinstance(intent, OperationalIntent):
            raise TypeError("Expected an OperationalIntent message")
        context_id = context_id.strip()
        with self._lock:
            if self._software_stop_requested_locally or (
                self._software_stop_state is not None and self._software_stop_state.latched
            ):
                self._reject_operation(context_id, "Software stop is latched; confirmed operator reset is required")
                return None
        if not self._operation_client.server_is_ready():
            self._reject_operation(
                context_id, "Application operation server is unavailable"
            )
            return None
        goal = ExecuteOperation.Goal()
        goal.client_id = self.client_id
        goal.context_id = context_id
        goal.intent = deepcopy(intent)
        local_id = uuid4().hex
        with self._lock:
            self._pending_operations[local_id] = _PendingOperation(context_id, self._monotonic_clock())
        try:
            future = self._operation_client.send_goal_async(
                goal,
                feedback_callback=partial(self._receive_feedback, local_id),
            )
        except Exception as exception:
            self._mark_uncertain(local_id, context_id, str(exception))
            return local_id
        future.add_done_callback(
            partial(self._receive_goal_response, local_id, context_id)
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
        """Latch the application software stop and request cancellation."""
        with self._lock:
            self._software_stop_requested_locally = True
            self._software_stop_observed_latched = False
            self._software_stop_request_id = ""
            self._software_stop_generation += 1
            generation = self._software_stop_generation
            self.software_stop_state_changed.emit(SoftwareStopState(
                latched=True, stop_confirmed=False,
                detail="Software stop requested; stopping is unconfirmed",
            ))
        if not self._emergency_client.service_is_ready():
            self.emergency_stop_finished.emit(
                False,
                "Software-stop service is unavailable; stopping is unconfirmed",
            )
            return None
        request = EmergencyStop.Request()
        request.client_id = self.client_id
        try:
            future = self._emergency_client.call_async(request)
        except Exception as exception:
            self.emergency_stop_finished.emit(False, str(exception))
            return None
        future.add_done_callback(partial(self._receive_emergency_result, generation))
        return future

    def reset_software_stop(self):
        """Request a motion-free reset; retain the latch until owner feedback."""
        if not self._reset_software_stop_client.service_is_ready():
            self.software_stop_reset_finished.emit(False, "Software-stop reset service is unavailable")
            return None
        request = ResetSoftwareStop.Request()
        request.client_id = self.client_id
        try:
            future = self._reset_software_stop_client.call_async(request)
        except Exception as exception:
            self.software_stop_reset_finished.emit(False, str(exception))
            return None
        future.add_done_callback(self._receive_software_stop_reset_result)
        return future

    def _receive_software_stop_state(self, state):
        with self._lock:
            sequence = int(state.status_sequence)
            if sequence <= self._software_stop_sequence:
                return
            # A retained pre-stop idle sample must not undo a local stop before
            # the owner has acknowledged that latch.
            if (self._software_stop_requested_locally and not state.latched
                    and (not self._software_stop_observed_latched
                         or state.request_id != self._software_stop_request_id)):
                return
            self._software_stop_sequence = sequence
            self._software_stop_state = deepcopy(state)
            if state.latched and (not self._software_stop_requested_locally
                                  or state.request_id == self._software_stop_request_id):
                self._software_stop_observed_latched = True
            if not state.latched:
                self._software_stop_requested_locally = False
                self._software_stop_observed_latched = False
            self.software_stop_state_changed.emit(state)

    def _receive_software_stop_reset_result(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.software_stop_reset_finished.emit(False, str(exception))
            return
        self.software_stop_reset_finished.emit(bool(response.accepted), response.detail)

    def _receive_goal_response(self, local_id, context_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self._mark_uncertain(local_id, context_id, str(exception))
            return
        if not goal_handle.accepted:
            with self._lock:
                pending = self._pending_operations.pop(local_id, None)
                if pending is not None:
                    self._reject_operation(context_id, "Operational request was rejected")
            return
        with self._lock:
            pending = self._pending_operations.get(local_id)
            if pending is not None:
                pending.goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
        except Exception as exception:
            self._mark_uncertain(local_id, context_id, str(exception))
            return
        result_future.add_done_callback(
            partial(self._receive_result, local_id, context_id)
        )

    def _receive_feedback(self, local_id, feedback_message):
        self._emit_state(feedback_message.feedback.state, local_id)

    def _receive_result(self, local_id, context_id, future):
        try:
            result = future.result().result
        except Exception as exception:
            # Losing the result after dispatch does not prove execution ended.
            self._mark_uncertain(local_id, context_id, str(exception))
            return
        self._emit_state(result.state, local_id)

    def _reject_operation(self, context_id, detail):
        self.operation_rejected.emit(context_id, detail)
        self.request_rejected.emit(detail)

    def _receive_state(self, state):
        self._emit_state(state)

    def _emit_state(self, state, local_id=None):
        with self._lock:
            return self._apply_state_locked(state, local_id)

    def _apply_state_locked(self, state, local_id):
        fresh = self._state_order.accept(state)
        if fresh:
            observed_at = self._monotonic_clock()
            self._observed_states[state.request_id] = (deepcopy(state), observed_at)
        else:
            observed = self._observed_states.get(state.request_id)
            if local_id is None or observed is None or observed[0] != state:
                return False
            # The topic may arrive before action feedback establishes the local
            # request identity. Correlate that exact observation without giving
            # a duplicate a new heartbeat or clearing existing uncertainty.
            observed_at = observed[1]
        terminal = state.state in {
            state.STATE_SUCCEEDED, state.STATE_FAILED,
            state.STATE_CANCELLED, state.STATE_STOP_CONFIRMED,
        }
        if state.client_id == self.client_id:
            candidates = [
                (key, pending) for key, pending in self._pending_operations.items()
                if (key == local_id or pending.request_id == state.request_id
                    or (not pending.request_id and state.context_id
                        and pending.context_id == state.context_id))
                and pending.context_id == state.context_id
                and (not pending.request_id or pending.request_id == state.request_id)
            ]
            if len(candidates) == 1:
                key, pending = candidates[0]
                if fresh or not pending.request_id:
                    pending.last_seen = observed_at
                pending.request_id = state.request_id
                if fresh:
                    pending.uncertain = False
                if terminal:
                    self._pending_operations.pop(key, None)
        if not fresh:
            return True
        fingerprint = (
            state.request_id,
            state.execution_instance_id,
            int(state.status_sequence),
            int(state.state),
            state.detail,
            int(state.buffered_command_count),
        )
        if fingerprint == self._last_state_fingerprint:
            return True
        self._last_state_fingerprint = fingerprint
        self.state_changed.emit(state)
        return True

    def _mark_uncertain(self, local_id, context_id, detail):
        with self._lock:
            pending = self._pending_operations.get(local_id)
            if pending is None:
                return
            pending.uncertain = True
            self.operation_uncertain.emit(context_id, detail)

    def _poll_liveness(self):
        with self._lock:
            now = self._monotonic_clock()
            for local_id, pending in tuple(self._pending_operations.items()):
                timeout = (self._lifecycle.heartbeat_timeout_sec if pending.request_id
                           else self._lifecycle.ack_timeout_sec)
                if not pending.uncertain and now - pending.last_seen >= timeout:
                    self._mark_uncertain(
                        local_id, pending.context_id,
                        "Application state updates stopped; execution remains unconfirmed",
                    )

    def _receive_cancellation_result(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if not response.accepted:
            self.request_rejected.emit(response.detail)

    def _receive_emergency_result(self, generation, future):
        try:
            response = future.result()
        except Exception as exception:
            self.emergency_stop_finished.emit(False, str(exception))
            return
        with self._lock:
            if generation != self._software_stop_generation:
                return
            if response.accepted:
                self._software_stop_request_id = response.request_id
                state = self._software_stop_state
                self._software_stop_observed_latched = bool(
                    state is not None and state.latched and state.request_id == response.request_id
                )
        self.emergency_stop_finished.emit(
            bool(response.accepted),
            response.detail,
        )

    def destroy(self):
        """Destroy client-side ROS entities owned by the adapter."""
        self._operation_client.destroy()
        self._liveness_timer.cancel()
        self.node.destroy_timer(self._liveness_timer)
        self.node.destroy_client(self._cancel_client)
        self.node.destroy_client(self._emergency_client)
        self.node.destroy_client(self._reset_software_stop_client)
        self.node.destroy_subscription(self._state_subscription)
        self.node.destroy_subscription(self._software_stop_subscription)
        with self._lock:
            self._pending_operations.clear()
            self._observed_states.clear()

    @staticmethod
    def _client_id(value):
        normalized = value.strip() if isinstance(value, str) else ""
        return normalized or f"fault_detector_ui_{uuid4().hex}"
