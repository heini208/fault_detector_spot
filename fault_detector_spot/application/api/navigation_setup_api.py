"""Expose navigation setup through typed ROS action interfaces."""

from dataclasses import dataclass, field
from threading import Event, RLock
from uuid import uuid4

from fault_detector_msgs.action import ExecuteNavigationSetup
from fault_detector_msgs.msg import (
    NavigationSetupIntent,
    NavigationSetupState,
)
from fault_detector_msgs.srv import CloseSetup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    MODE_LOCALIZATION,
    MODE_MAPPING,
    NavigationSetupCoordinator,
    NavigationSetupSnapshot,
    NavigationSetupStatus,
)
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS


_RUNTIME_OPERATIONS = {
    NavigationSetupIntent.OPERATION_START_MAPPING: CommandID.START_SLAM,
    NavigationSetupIntent.OPERATION_START_LOCALIZATION: (
        CommandID.START_LOCALIZATION
    ),
    NavigationSetupIntent.OPERATION_STOP_MAPPING: CommandID.STOP_MAPPING,
}


@dataclass
class _NavigationExecution:
    goal_handle: object
    request_id: str
    context: object
    operation_code: int
    finished: Event = field(default_factory=Event)
    state: object = None
    cancellation_requested: bool = False


class NavigationSetupApi:
    """Own ROS transport for one navigation setup coordinator."""

    def __init__(self, node, coordinator: NavigationSetupCoordinator):
        self.node = node
        self.coordinator = coordinator
        self._lock = RLock()
        self._executions = {}
        self._early_states = {}
        self._callback_group = ReentrantCallbackGroup()
        self._transaction_handlers = {
            NavigationSetupIntent.OPERATION_CREATE_MAP_DEFINITION: (
                self._create_map_definition
            ),
            NavigationSetupIntent.OPERATION_DELETE_MAP: self._delete_map,
            NavigationSetupIntent.OPERATION_SELECT_MAP: self._select_map,
            NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT: (
                self._add_current_waypoint
            ),
            NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK: (
                self._add_visible_tag_landmark
            ),
            NavigationSetupIntent.OPERATION_DELETE_WAYPOINT: (
                self._delete_waypoint
            ),
            NavigationSetupIntent.OPERATION_DELETE_LANDMARK: (
                self._delete_landmark
            ),
        }
        self._state_publisher = node.create_publisher(
            NavigationSetupState,
            "fault_detector/application/navigation_setup_state",
            APPLICATION_STATE_QOS,
        )
        self._action_server = ActionServer(
            node,
            ExecuteNavigationSetup,
            "fault_detector/application/execute_navigation_setup",
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )
        self._close_service = node.create_service(
            CloseSetup,
            "fault_detector/application/close_navigation_setup",
            self._close,
            callback_group=self._callback_group,
        )
        coordinator.add_status_listener(self._receive_status)

    def _accept(self, goal_request):
        try:
            required_client_id(goal_request.client_id)
            self._validate_operation(goal_request.intent.operation)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        operation_code = int(goal.intent.operation)
        try:
            if operation_code == NavigationSetupIntent.OPERATION_OPEN:
                return self._open_context(goal_handle, goal)
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
        except Exception as exception:
            return self._abort(goal_handle, goal, str(exception))
        return self._execute_in_context(goal_handle, goal, context)

    def _open_context(self, goal_handle, goal):
        snapshot = self.coordinator.open_context(goal.client_id)
        return self._succeed(
            goal_handle,
            self._state(
                snapshot,
                NavigationSetupIntent.OPERATION_OPEN,
                NavigationSetupState.STATE_READY,
                "Navigation setup opened",
            ),
        )

    def _execute_in_context(self, goal_handle, goal, context):
        operation_code = int(goal.intent.operation)
        try:
            if operation_code in _RUNTIME_OPERATIONS:
                return self._execute_runtime(
                    goal_handle,
                    context,
                    goal.intent,
                )
            snapshot = self._execute_transaction(context, goal.intent)
            return self._succeed(
                goal_handle,
                self._state(
                    snapshot,
                    operation_code,
                    NavigationSetupState.STATE_SUCCEEDED,
                    self._transaction_detail(operation_code),
                ),
            )
        except Exception as exception:
            return self._abort(
                goal_handle,
                goal,
                str(exception),
                context,
            )

    def _execute_transaction(self, context, intent):
        try:
            handler = self._transaction_handlers[int(intent.operation)]
        except KeyError as exception:
            raise ValueError(
                f"Unsupported navigation operation: {intent.operation}"
            ) from exception
        return handler(context, intent)

    def _create_map_definition(self, context, intent):
        return self.coordinator.create_and_select_map(
            context,
            intent.map_name,
        )

    def _delete_map(self, context, intent):
        return self.coordinator.delete_map(context, intent.map_name)

    def _select_map(self, context, intent):
        return self.coordinator.select_map(context, intent.map_name)

    def _add_current_waypoint(self, context, intent):
        return self.coordinator.add_current_waypoint(
            context,
            intent.map_name,
            intent.waypoint_name,
        )

    def _add_visible_tag_landmark(self, context, intent):
        return self.coordinator.add_visible_tag_landmark(
            context,
            intent.map_name,
            int(intent.tag_id),
        )

    def _delete_waypoint(self, context, intent):
        return self.coordinator.delete_waypoint(
            context,
            intent.map_name,
            intent.waypoint_name,
        )

    def _delete_landmark(self, context, intent):
        return self.coordinator.delete_landmark(
            context,
            intent.map_name,
            intent.waypoint_name,
        )

    def _execute_runtime(self, goal_handle, context, intent):
        operation_code = int(intent.operation)
        operation = self.coordinator.submit_runtime_operation(
            context=context,
            operation_code=operation_code,
            command_id=_RUNTIME_OPERATIONS[operation_code],
            map_name=intent.map_name,
        )
        execution = _NavigationExecution(
            goal_handle=goal_handle,
            request_id=operation.request_id,
            context=context,
            operation_code=operation_code,
        )
        with self._lock:
            self._executions[operation.request_id] = execution
            early_state = self._early_states.pop(
                operation.request_id,
                None,
            )
        if early_state is not None:
            execution.state = early_state
            feedback = ExecuteNavigationSetup.Feedback()
            feedback.state = early_state
            goal_handle.publish_feedback(feedback)
            if early_state.state in {
                NavigationSetupState.STATE_SUCCEEDED,
                NavigationSetupState.STATE_FAILED,
                NavigationSetupState.STATE_CANCELLED,
            }:
                execution.finished.set()
        self._wait_for_runtime_execution(execution)
        return self._runtime_result(execution)

    def _wait_for_runtime_execution(self, execution):
        goal_handle = execution.goal_handle
        while not execution.finished.wait(0.05):
            if (
                goal_handle.is_cancel_requested
                and not execution.cancellation_requested
            ):
                execution.cancellation_requested = True
                try:
                    self.coordinator.cancel(
                        execution.context,
                        execution.request_id,
                    )
                except LookupError:
                    pass

    def _runtime_result(self, execution):
        state = execution.state
        with self._lock:
            self._executions.pop(execution.request_id, None)
        if state is None:
            state = self._state(
                self.coordinator.snapshot(execution.context),
                execution.operation_code,
                NavigationSetupState.STATE_FAILED,
                "Runtime operation ended without a state",
                execution.request_id,
            )
        result = ExecuteNavigationSetup.Result()
        result.state = state
        if state.state == NavigationSetupState.STATE_SUCCEEDED:
            execution.goal_handle.succeed()
        elif (
            state.state == NavigationSetupState.STATE_CANCELLED
            and execution.goal_handle.is_cancel_requested
        ):
            execution.goal_handle.canceled()
        else:
            execution.goal_handle.abort()
        return result

    def _receive_status(self, status: NavigationSetupStatus) -> None:
        state = self._state(
            status.snapshot,
            status.operation_code,
            self._public_state(status.state),
            status.detail,
            status.request_id,
        )
        self._state_publisher.publish(state)
        self.node.get_logger().info(
            "Navigation setup request "
            f"{status.request_id}: {status.state.value}: "
            f"{status.detail}"
        )
        with self._lock:
            execution = self._executions.get(status.request_id)
            if execution is None:
                self._early_states[status.request_id] = state
                return
        execution.state = state
        feedback = ExecuteNavigationSetup.Feedback()
        feedback.state = state
        execution.goal_handle.publish_feedback(feedback)
        if status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }:
            execution.finished.set()

    def _close(self, request, response):
        try:
            context = self.coordinator.context(
                request.context_id,
                request.client_id,
            )
            self.coordinator.close_context(context)
        except (TypeError, ValueError, LookupError) as exception:
            response.closed = False
            response.detail = str(exception)
            return response
        response.closed = True
        response.detail = "Navigation setup closed"
        return response

    def _succeed(self, goal_handle, state):
        self._state_publisher.publish(state)
        result = ExecuteNavigationSetup.Result()
        result.state = state
        goal_handle.succeed()
        return result

    def _abort(self, goal_handle, goal, detail, context=None):
        if context is not None and self.coordinator.setup_coordinator.is_current(
            context
        ):
            state = self._state(
                self.coordinator.snapshot(context),
                int(goal.intent.operation),
                NavigationSetupState.STATE_FAILED,
                detail,
            )
            self._state_publisher.publish(state)
            result = ExecuteNavigationSetup.Result()
            result.state = state
            goal_handle.abort()
            return result
        state = NavigationSetupState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.request_id = uuid4().hex
        state.client_id = goal.client_id.strip()
        state.context_id = goal.context_id.strip()
        state.operation = int(goal.intent.operation)
        state.state = NavigationSetupState.STATE_FAILED
        state.detail = detail
        self._state_publisher.publish(state)
        result = ExecuteNavigationSetup.Result()
        result.state = state
        goal_handle.abort()
        return result

    def _state(
        self,
        snapshot: NavigationSetupSnapshot,
        operation_code: int,
        state_code: int,
        detail: str,
        request_id: str = "",
    ):
        message = NavigationSetupState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = request_id or uuid4().hex
        message.client_id = snapshot.context.client_id
        message.context_id = snapshot.context.context_id
        message.revision = snapshot.context.revision
        message.operation = int(operation_code)
        message.state = int(state_code)
        message.detail = detail
        message.active_map = snapshot.active_map
        message.mode = self._mode_code(snapshot.mode)
        message.map_names = list(snapshot.map_names)
        message.waypoint_names = list(snapshot.waypoint_names)
        message.landmark_names = list(snapshot.landmark_names)
        return message

    @staticmethod
    def _transaction_detail(operation_code: int) -> str:
        details = {
            NavigationSetupIntent.OPERATION_CREATE_MAP_DEFINITION: (
                "Map created and selected"
            ),
            NavigationSetupIntent.OPERATION_SELECT_MAP: "Map selected",
            NavigationSetupIntent.OPERATION_DELETE_MAP: "Map deleted",
            NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT: (
                "Waypoint saved"
            ),
            NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK: (
                "Landmark saved"
            ),
            NavigationSetupIntent.OPERATION_DELETE_WAYPOINT: (
                "Waypoint deleted"
            ),
            NavigationSetupIntent.OPERATION_DELETE_LANDMARK: (
                "Landmark deleted"
            ),
        }
        return details.get(
            int(operation_code),
            "Navigation setup transaction succeeded",
        )

    @staticmethod
    def _validate_operation(operation_code: int) -> None:
        supported = {
            value
            for name, value in vars(NavigationSetupIntent).items()
            if name.startswith("OPERATION_")
            and name != "OPERATION_UNSPECIFIED"
        }
        if int(operation_code) not in supported:
            raise ValueError(
                f"Unsupported navigation operation: {operation_code}"
            )

    @staticmethod
    def _public_state(state: CommandControllerState) -> int:
        return {
            CommandControllerState.QUEUED: NavigationSetupState.STATE_QUEUED,
            CommandControllerState.DISPATCHED: (
                NavigationSetupState.STATE_RUNNING
            ),
            CommandControllerState.RUNNING: NavigationSetupState.STATE_RUNNING,
            CommandControllerState.SUCCEEDED: (
                NavigationSetupState.STATE_SUCCEEDED
            ),
            CommandControllerState.FAILED: NavigationSetupState.STATE_FAILED,
            CommandControllerState.CANCELLED: (
                NavigationSetupState.STATE_CANCELLED
            ),
        }[state]

    @staticmethod
    def _mode_code(mode: str) -> int:
        if mode == MODE_MAPPING:
            return NavigationSetupState.MODE_MAPPING
        if mode == MODE_LOCALIZATION:
            return NavigationSetupState.MODE_LOCALIZATION
        return NavigationSetupState.MODE_NONE

    def close(self) -> None:
        """Detach transport resources from the coordinator."""
        self.coordinator.remove_status_listener(self._receive_status)
        self._action_server.destroy()


__all__ = ["NavigationSetupApi"]
