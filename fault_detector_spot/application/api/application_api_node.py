"""Expose the application command boundary as typed ROS interfaces."""

from dataclasses import dataclass, field
from threading import Event, RLock
from typing import Dict, Optional

import rclpy
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState
from fault_detector_msgs.srv import CancelOperation, EmergencyStop
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandControllerStatus,
    UnknownCommandRequest,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


_TERMINAL_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
})


def _required_client_id(value: str) -> str:
    if not isinstance(value, str):
        raise TypeError("Client ID must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError("Client ID must not be empty")
    return normalized


@dataclass
class _OperationExecution:
    goal_handle: object
    intent: int
    request: CommandRequest
    finished: Event = field(default_factory=Event)
    state: Optional[ApplicationCommandState] = None
    cancellation_requested: bool = False


class ApplicationApiNode(Node):
    """Own the remote operational API and one command controller."""

    def __init__(self):
        super().__init__("application_api")
        self._lock = RLock()
        self._executions: Dict[str, _OperationExecution] = {}
        self._callback_group = ReentrantCallbackGroup()
        self.command_controller = CommandController(self)
        self.command_controller.add_status_listener(
            self._handle_controller_status
        )
        self._state_publisher = self.create_publisher(
            ApplicationCommandState,
            "fault_detector/application/command_state",
            APPLICATION_STATE_QOS,
        )
        self._operation_server = ActionServer(
            self,
            ExecuteOperation,
            "fault_detector/application/execute_operation",
            execute_callback=self._execute_operation,
            goal_callback=self._accept_operation,
            cancel_callback=self._accept_cancellation,
            callback_group=self._callback_group,
        )
        self._cancel_service = self.create_service(
            CancelOperation,
            "fault_detector/application/cancel_operation",
            self._cancel_operation,
            callback_group=self._callback_group,
        )
        self._emergency_service = self.create_service(
            EmergencyStop,
            "fault_detector/application/emergency_stop",
            self._emergency_stop,
            callback_group=self._callback_group,
        )

    def _accept_operation(self, goal_request):
        try:
            _required_client_id(goal_request.client_id)
            operational_intent_to_command(goal_request.intent)
        except (TypeError, ValueError) as exception:
            self.get_logger().error(
                f"Rejected operational intent: {exception}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancellation(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute_operation(self, goal_handle):
        goal = goal_handle.request
        command = operational_intent_to_command(goal.intent)
        request = CommandRequest.create(
            command=command,
            client_id=_required_client_id(goal.client_id),
            context_id=goal.context_id.strip(),
            origin=CommandOrigin.OPERATIONAL,
            recording_policy=(
                RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
            ),
        )
        execution = _OperationExecution(
            goal_handle=goal_handle,
            intent=int(goal.intent.intent),
            request=request,
        )
        with self._lock:
            self._executions[request.request_id] = execution
        try:
            self.command_controller.submit(request)
        except Exception as exception:
            state = self._failure_state(execution, str(exception))
            execution.state = state
            execution.finished.set()

        while not execution.finished.wait(0.05):
            if (
                goal_handle.is_cancel_requested
                and not execution.cancellation_requested
            ):
                execution.cancellation_requested = True
                try:
                    self.command_controller.cancel(request.request_id)
                except UnknownCommandRequest:
                    pass

        state = execution.state or self._failure_state(
            execution,
            "Operation ended without a terminal state",
        )
        result = ExecuteOperation.Result()
        result.state = state
        if state.state == ApplicationCommandState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif state.state == ApplicationCommandState.STATE_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        with self._lock:
            self._executions.pop(request.request_id, None)
        return result

    def _cancel_operation(self, request, response):
        try:
            client_id = _required_client_id(request.client_id)
            with self._lock:
                execution = self._executions.get(request.request_id)
            if (
                execution is not None
                and execution.request.client_id != client_id
            ):
                raise ValueError(
                    "Client ID does not own the requested operation"
                )
            cancellation_id = self.command_controller.cancel(
                request.request_id
            )
        except (TypeError, ValueError, UnknownCommandRequest) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.cancellation_request_id = cancellation_id
        response.detail = "Cancellation accepted"
        return response

    def _emergency_stop(self, request, response):
        try:
            client_id = _required_client_id(request.client_id)
            request_id = self.command_controller.cancel_all(client_id)
        except (TypeError, ValueError) as exception:
            response.accepted = False
            response.detail = str(exception)
            return response
        response.accepted = True
        response.request_id = request_id
        response.detail = "Emergency stop accepted"
        return response

    def _handle_controller_status(
        self,
        status: CommandControllerStatus,
    ) -> None:
        with self._lock:
            execution = self._executions.get(status.request_id)
        if execution is None:
            return
        state = self._state_message(execution, status)
        execution.state = state
        self._state_publisher.publish(state)
        feedback = ExecuteOperation.Feedback()
        feedback.state = state
        execution.goal_handle.publish_feedback(feedback)
        if status.state in _TERMINAL_STATES:
            execution.finished.set()

    def _state_message(self, execution, status):
        message = ApplicationCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = status.request_id
        message.client_id = status.request.client_id
        message.context_id = status.request.context_id
        message.intent = execution.intent
        message.state = self._public_state(status.state)
        message.detail = status.detail or status.state.value
        message.buffered_command_count = status.buffered_command_count
        return message

    def _failure_state(self, execution, detail):
        message = ApplicationCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = execution.request.request_id
        message.client_id = execution.request.client_id
        message.context_id = execution.request.context_id
        message.intent = execution.intent
        message.state = ApplicationCommandState.STATE_FAILED
        message.detail = detail
        self._state_publisher.publish(message)
        return message

    @staticmethod
    def _public_state(state):
        values = {
            CommandControllerState.QUEUED: (
                ApplicationCommandState.STATE_QUEUED
            ),
            CommandControllerState.DISPATCHED: (
                ApplicationCommandState.STATE_DISPATCHED
            ),
            CommandControllerState.RUNNING: (
                ApplicationCommandState.STATE_RUNNING
            ),
            CommandControllerState.SUCCEEDED: (
                ApplicationCommandState.STATE_SUCCEEDED
            ),
            CommandControllerState.FAILED: (
                ApplicationCommandState.STATE_FAILED
            ),
            CommandControllerState.CANCELLED: (
                ApplicationCommandState.STATE_CANCELLED
            ),
        }
        return values[state]

    def destroy_node(self):
        self.command_controller.remove_status_listener(
            self._handle_controller_status
        )
        self._operation_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApplicationApiNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
