"""Provide the PyQt UI with the typed probe setup API."""

from copy import deepcopy
from functools import partial
from uuid import uuid4

from PyQt5.QtCore import QObject, pyqtSignal
from fault_detector_msgs.action import (
    ExecuteProbeSetupMotion,
    ExecuteProbeSurfaceVerification,
    FinalizeProbeRefinement,
)
from fault_detector_msgs.msg import (
    ProbeSetupIntent,
    ProbeSetupMotionIntent,
    ProbeSetupState,
)
from fault_detector_msgs.srv import (
    CloseProbeSetup,
    ExecuteProbeSetup,
    GetProbeReferencePreview,
)
from rclpy.action import ActionClient

from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


class ProbeSetupClient(QObject):
    """Submit probe authoring intent and expose immutable state."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)
    close_finished = pyqtSignal(bool, str)
    preview_received = pyqtSignal(object)
    preview_rejected = pyqtSignal(str, str)

    def __init__(self, node, client_id: str):
        super().__init__()
        self.node = node
        self.client_id = client_id
        self.context_id = ""
        self._last_state_fingerprint = None
        self._pending_request_id = ""
        self._motion_goal_handles = {}
        self._surface_goal_handles = {}
        self._finalization_goal_handles = {}
        self._execute_client = node.create_client(
            ExecuteProbeSetup,
            "fault_detector/application/execute_probe_setup",
        )
        self._close_client = node.create_client(
            CloseProbeSetup,
            "fault_detector/application/close_probe_setup",
        )
        self._preview_client = node.create_client(
            GetProbeReferencePreview,
            "fault_detector/application/get_probe_reference_preview",
        )
        self._motion_client = ActionClient(
            node,
            ExecuteProbeSetupMotion,
            "fault_detector/application/execute_probe_setup_motion",
        )
        self._surface_client = ActionClient(
            node,
            ExecuteProbeSurfaceVerification,
            "fault_detector/application/execute_probe_surface_verification",
        )
        self._finalization_client = ActionClient(
            node,
            FinalizeProbeRefinement,
            "fault_detector/application/finalize_probe_refinement",
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

    def execute_motion(self, intent: ProbeSetupMotionIntent):
        """Submit one single-step setup movement in the current context."""
        if not isinstance(intent, ProbeSetupMotionIntent):
            raise TypeError("Expected a ProbeSetupMotionIntent message")
        if not self.context_id:
            self.request_rejected.emit("Probe setup context is not open")
            return None
        if not self._motion_client.server_is_ready():
            return None
        goal = ExecuteProbeSetupMotion.Goal()
        goal.client_id = self.client_id
        goal.context_id = self.context_id
        goal.intent = deepcopy(intent)
        local_id = uuid4().hex
        future = self._motion_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._receive_motion_feedback,
                local_id,
            ),
        )
        future.add_done_callback(
            partial(self._receive_motion_goal, local_id)
        )
        return local_id

    def execute_surface_verification(self):
        """Request one server-owned closed-loop surface verification."""
        if not self.context_id:
            self.request_rejected.emit("Probe setup context is not open")
            return None
        if self._surface_goal_handles:
            self.request_rejected.emit(
                "Surface verification is already in progress"
            )
            return None
        if not self._surface_client.server_is_ready():
            return None
        goal = ExecuteProbeSurfaceVerification.Goal()
        goal.client_id = self.client_id
        goal.context_id = self.context_id
        local_id = uuid4().hex
        self._surface_goal_handles[local_id] = None
        future = self._surface_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._receive_surface_feedback,
                local_id,
            ),
        )
        future.add_done_callback(
            partial(self._receive_surface_goal, local_id)
        )
        return local_id

    def finalize_refinement(
        self,
        save_requested: bool,
        probe_point_id: str = "",
        probe_point_display_name: str = "",
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.087,
        measurement_duration_sec: float = 1.0,
    ):
        """Request server-owned persistence and mandatory retraction."""
        if not isinstance(save_requested, bool):
            raise TypeError("Save requested flag must be a boolean")
        if not self.context_id:
            self.request_rejected.emit("Probe setup context is not open")
            return None
        if self._finalization_goal_handles:
            self.request_rejected.emit(
                "Probe refinement finalization is already in progress"
            )
            return None
        if not self._finalization_client.server_is_ready():
            return None
        goal = FinalizeProbeRefinement.Goal()
        goal.client_id = self.client_id
        goal.context_id = self.context_id
        goal.mode = (
            FinalizeProbeRefinement.Goal.MODE_SAVE_AND_RETRACT
            if save_requested
            else FinalizeProbeRefinement.Goal.MODE_RETRACT_WITHOUT_SAVING
        )
        goal.probe_point_id = probe_point_id
        goal.probe_point_display_name = probe_point_display_name
        goal.position_tolerance_m = float(position_tolerance_m)
        goal.orientation_tolerance_rad = float(
            orientation_tolerance_rad
        )
        goal.measurement_duration_sec = float(measurement_duration_sec)
        local_id = uuid4().hex
        self._finalization_goal_handles[local_id] = None
        future = self._finalization_client.send_goal_async(
            goal,
            feedback_callback=partial(
                self._receive_finalization_feedback,
                local_id,
            ),
        )
        future.add_done_callback(
            partial(self._receive_finalization_goal, local_id)
        )
        return local_id

    def close(self):
        """Close the current server-owned probe setup context."""
        if not self.context_id:
            return None
        if (
            self._pending_request_id
            or self._motion_goal_handles
            or self._surface_goal_handles
            or self._finalization_goal_handles
        ):
            self.close_finished.emit(
                False,
                "Probe setup operation is still in progress",
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

    def request_preview(self, reference_view_id: str):
        """Request one selected context's read-only RGB preview."""
        view_id = reference_view_id.strip()
        if not self.context_id:
            self.preview_rejected.emit(
                view_id,
                "Probe setup context is not open",
            )
            return None
        if not view_id:
            self.preview_rejected.emit(
                view_id,
                "Reference view ID must not be empty",
            )
            return None
        if not self._preview_client.service_is_ready():
            return None
        request = GetProbeReferencePreview.Request()
        request.client_id = self.client_id
        request.context_id = self.context_id
        request.reference_view_id = view_id
        future = self._preview_client.call_async(request)
        future.add_done_callback(
            partial(self._receive_preview, view_id)
        )
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
            state.surface_verification_request_id,
            int(state.surface_verification_state),
            bool(state.has_surface_distance_measurement),
            float(state.measured_surface_distance_m),
            float(state.surface_distance_error_m),
            int(state.surface_verification_iteration),
            bool(state.surface_recovery_required),
            bool(state.refinement_recovery_required),
            state.refinement_recovery_message,
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

    def _receive_preview(self, view_id, future):
        try:
            response = future.result()
        except Exception as exception:
            self.preview_rejected.emit(view_id, str(exception))
            return
        if not response.success:
            self.preview_rejected.emit(view_id, response.detail)
            return
        if response.reference_view_id != view_id:
            self.preview_rejected.emit(
                view_id,
                "Reference preview response does not match its request",
            )
            return
        self.preview_received.emit(response)

    def _receive_motion_goal(self, local_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if not goal_handle.accepted:
            self.request_rejected.emit("Probe setup motion was rejected")
            return
        self._motion_goal_handles[local_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._receive_motion_result, local_id)
        )

    def _receive_motion_feedback(self, _local_id, feedback_message):
        self._emit_state(feedback_message.feedback.state)

    def _receive_motion_result(self, local_id, future):
        self._motion_goal_handles.pop(local_id, None)
        try:
            result = future.result().result
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        self._emit_state(result.state)

    def _receive_surface_goal(self, local_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self._surface_goal_handles.pop(local_id, None)
            self.request_rejected.emit(str(exception))
            return
        if not goal_handle.accepted:
            self._surface_goal_handles.pop(local_id, None)
            self.request_rejected.emit("Surface verification was rejected")
            return
        self._surface_goal_handles[local_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._receive_surface_result, local_id)
        )

    def _receive_surface_feedback(self, _local_id, feedback_message):
        self._emit_state(feedback_message.feedback.state)

    def _receive_surface_result(self, local_id, future):
        self._surface_goal_handles.pop(local_id, None)
        try:
            result = future.result().result
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        self._emit_state(result.state)

    def _receive_finalization_goal(self, local_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self._finalization_goal_handles.pop(local_id, None)
            self.request_rejected.emit(str(exception))
            return
        if not goal_handle.accepted:
            self._finalization_goal_handles.pop(local_id, None)
            self.request_rejected.emit(
                "Probe refinement finalization was rejected"
            )
            return
        self._finalization_goal_handles[local_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._receive_finalization_result, local_id)
        )

    def _receive_finalization_feedback(
        self,
        _local_id,
        feedback_message,
    ):
        self._emit_state(feedback_message.feedback.state)

    def _receive_finalization_result(self, local_id, future):
        self._finalization_goal_handles.pop(local_id, None)
        try:
            result = future.result().result
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        self._emit_state(result.state)

    def destroy(self):
        """Destroy client-side ROS resources owned by this adapter."""
        self.node.destroy_client(self._execute_client)
        self.node.destroy_client(self._close_client)
        self.node.destroy_client(self._preview_client)
        self._motion_client.destroy()
        self._surface_client.destroy()
        self._finalization_client.destroy()
        self.node.destroy_subscription(self._state_subscription)
        self._motion_goal_handles.clear()
        self._surface_goal_handles.clear()
        self._finalization_goal_handles.clear()


__all__ = ["ProbeSetupClient"]
