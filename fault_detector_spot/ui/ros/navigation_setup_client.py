"""Provide the PyQt UI with the typed navigation setup API."""

from copy import deepcopy
from functools import partial
from uuid import uuid4

from PyQt5.QtCore import QObject, pyqtSignal

from fault_detector_msgs.action import ExecuteNavigationSetup
from fault_detector_msgs.msg import NavigationSetupIntent, NavigationSetupState
from fault_detector_msgs.srv import CloseNavigationSetup
from rclpy.action import ActionClient

from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


_RUNTIME_OPERATIONS = frozenset({
    NavigationSetupIntent.OPERATION_SELECT_MAP,
    NavigationSetupIntent.OPERATION_START_MAPPING,
    NavigationSetupIntent.OPERATION_START_LOCALIZATION,
    NavigationSetupIntent.OPERATION_STOP_MAPPING,
})


class NavigationSetupClient(QObject):
    """Submit navigation setup intent and expose immutable state."""

    state_changed = pyqtSignal(object)
    request_rejected = pyqtSignal(str)
    close_finished = pyqtSignal(bool, str)

    def __init__(self, node, client_id: str):
        super().__init__()
        self.node = node
        self.client_id = client_id
        self.context_id = ""
        self._last_state_fingerprint = None
        self._last_map_names = []
        self._goal_handles = {}
        self._action_client = ActionClient(
            node,
            ExecuteNavigationSetup,
            "fault_detector/application/execute_navigation_setup",
        )
        self._close_client = node.create_client(
            CloseNavigationSetup,
            "fault_detector/application/close_navigation_setup",
        )
        self._state_subscription = node.create_subscription(
            NavigationSetupState,
            "fault_detector/application/navigation_setup_state",
            self._receive_state,
            APPLICATION_STATE_QOS,
        )

    def open(self):
        """Open a navigation setup context when the server is ready."""
        intent = NavigationSetupIntent()
        intent.operation = NavigationSetupIntent.OPERATION_OPEN
        return self._send(intent, "")

    def execute(self, intent: NavigationSetupIntent):
        """Submit one navigation setup intent in the current context."""
        if not self.context_id:
            self.request_rejected.emit(
                "Navigation setup context is not open"
            )
            return None
        return self._send(intent, self.context_id)

    def close(self):
        """Close the current server-owned navigation setup context."""
        if not self.context_id:
            return None
        if not self._close_client.service_is_ready():
            self.close_finished.emit(
                False,
                "Navigation setup close service is unavailable",
            )
            return None
        request = CloseNavigationSetup.Request()
        request.client_id = self.client_id
        request.context_id = self.context_id
        future = self._close_client.call_async(request)
        future.add_done_callback(self._receive_close)
        return future

    def _send(self, intent, context_id):
        if not isinstance(intent, NavigationSetupIntent):
            raise TypeError("Expected a NavigationSetupIntent message")
        if not self._action_client.server_is_ready():
            return None
        goal = ExecuteNavigationSetup.Goal()
        goal.client_id = self.client_id
        goal.context_id = context_id
        goal.intent = deepcopy(intent)
        local_id = uuid4().hex
        future = self._action_client.send_goal_async(
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

    def _receive_goal_response(self, local_id, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self.request_rejected.emit(str(exception))
            return
        if not goal_handle.accepted:
            self.request_rejected.emit(
                "Navigation setup request was rejected"
            )
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

    def _emit_state(self, state):
        if state.client_id != self.client_id:
            return
        if self.context_id and state.context_id != self.context_id:
            return
        if state.context_id:
            self.context_id = state.context_id

        operation = int(state.operation)
        if state.map_names:
            self._last_map_names = list(state.map_names)
        elif operation in _RUNTIME_OPERATIONS and self._last_map_names:
            state = deepcopy(state)
            state.map_names = list(self._last_map_names)
        elif operation not in _RUNTIME_OPERATIONS:
            self._last_map_names = []

        fingerprint = (
            state.request_id,
            int(state.state),
            int(state.revision),
            state.detail,
            tuple(state.map_names),
        )
        if fingerprint == self._last_state_fingerprint:
            return
        self._last_state_fingerprint = fingerprint
        self.state_changed.emit(state)

    def _receive_state(self, state):
        self._emit_state(state)

    def _receive_close(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.close_finished.emit(False, str(exception))
            return
        if response.closed:
            self.context_id = ""
            self._last_map_names = []
        self.close_finished.emit(response.closed, response.detail)

    def destroy(self):
        """Destroy client-side ROS entities owned by this adapter."""
        self._action_client.destroy()
        self.node.destroy_client(self._close_client)
        self.node.destroy_subscription(self._state_subscription)
        self._goal_handles.clear()


__all__ = ["NavigationSetupClient"]
