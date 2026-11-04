"""Expose navigation setup through typed ROS action interfaces."""

from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.shutdown_callbacks import (
    ShutdownCallbackActivity, tracked_shutdown_callback, release_resources,
)

from dataclasses import dataclass, field
from threading import Event, RLock
from uuid import uuid4
import time

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.clock import Clock, ClockType

from fault_detector_msgs.action import ExecuteNavigationSetup
from fault_detector_msgs.msg import (
    NavigationSetupIntent,
    NavigationSetupState,
)
from fault_detector_msgs.srv import CloseSetup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

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
from fault_detector_spot.shared.ros.qos_profiles import APPLICATION_STATE_QOS, LATCHED_QOS
from fault_detector_spot.navigation.landmark_observations import NavigationObservationPending


_RUNTIME_OPERATIONS = {
    NavigationSetupIntent.OPERATION_START_MAPPING: CommandID.START_SLAM,
    NavigationSetupIntent.OPERATION_START_LOCALIZATION: (
        CommandID.START_LOCALIZATION
    ),
    NavigationSetupIntent.OPERATION_STOP_MAPPING: CommandID.STOP_MAPPING,
}
_TERMINAL_STATES = {
    NavigationSetupState.STATE_SUCCEEDED,
    NavigationSetupState.STATE_FAILED,
    NavigationSetupState.STATE_CANCELLED,
    NavigationSetupState.STATE_STOP_CONFIRMED,
}


@dataclass
class _NavigationExecution:
    goal_handle: object
    request_id: str
    context: object
    finished: Future = field(default_factory=Future)
    cancellation_requested: bool = False


class NavigationSetupApi:
    """Own ROS transport for one navigation setup coordinator."""

    def __init__(self, node, coordinator: NavigationSetupCoordinator, admission=None):
        self.node = node
        self.coordinator = coordinator
        self._lock = RLock()
        self._executions = {}
        self._admission = admission or OperationAdmission(limit=1)
        self._shutdown = Event()
        self._shutdown_callbacks = ShutdownCallbackActivity()
        self._early_states = {}
        self._runtime_submissions = 0
        self._pose_waiters = set()
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
        self._runtime_received_at = None
        self._runtime_subscription = node.create_subscription(
            DiagnosticStatus, "fault_detector/navigation_runtime",
            self._receive_runtime, LATCHED_QOS,
        )
        self._runtime_timer = node.create_timer(
            0.5, self._check_runtime_status,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        coordinator.observe_runtime(None, "", "Runtime status unavailable")

    def _receive_runtime(self, message):
        values = {item.key: item.value for item in message.values}
        mode = values.get("mode")
        if mode not in {"none", "mapping", "localization"}:
            return
        self._runtime_received_at = time.monotonic()
        self.coordinator.observe_runtime(
            mode, values.get("active_map", ""),
            message.message if message.level != DiagnosticStatus.OK else "",
        )

    def _check_runtime_status(self):
        if (self._runtime_received_at is None
                or time.monotonic() - self._runtime_received_at > 2.0):
            self.coordinator.observe_runtime(None, "", "Runtime status unavailable")

    def _accept(self, goal_request):
        if self._shutdown.is_set():
            return GoalResponse.REJECT
        try:
            required_client_id(goal_request.client_id)
            self._validate_operation(goal_request.intent.operation)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        if not self._admission.reserve(
            id(goal_request), context_id=goal_request.context_id.strip(),
            lane="physical",
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_cancel(self, goal_handle):
        self._admission.cancel(id(goal_handle.request))
        with self._lock:
            execution = next((
                candidate for candidate in self._executions.values()
                if candidate.goal_handle is goal_handle
            ), None)
        if execution is not None:
            self._request_execution_cancellation(execution)
        return CancelResponse.ACCEPT

    def _request_execution_cancellation(self, execution):
        with self._lock:
            if self._shutdown.is_set() or execution.cancellation_requested or execution.finished.done():
                return
            execution.cancellation_requested = True
        try:
            self.coordinator.cancel(execution.context, execution.request_id)
        except LookupError:
            pass

    @tracked_shutdown_callback
    async def _execute(self, goal_handle):
        goal = goal_handle.request
        key = id(goal)
        if not self._admission.claim(key):
            detail = ("Application is shutting down" if self._shutdown.is_set()
                      else "Operation admission is missing, cancelled, or already executing")
            try:
                return self._abort(goal_handle, goal, detail)
            finally:
                self._admission.release_pending(key)
        try:
            return await self._execute_admitted(goal_handle)
        finally:
            self._admission.release(key)

    async def _execute_admitted(self, goal_handle):
        goal = goal_handle.request
        if self._shutdown.is_set():
            return self._abort(goal_handle, goal, "Application is shutting down")
        if (goal_handle.is_cancel_requested
                or self._admission.cancel_requested(id(goal))):
            return self._abort(
                goal_handle, goal, "Navigation setup cancelled before dispatch", cancelled=True,
            )
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
        if operation_code in {
            NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK,
            NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT,
        }:
            return await self._execute_pose_save(goal_handle, context)
        if operation_code in _RUNTIME_OPERATIONS:
            try:
                return await self._execute_runtime(goal_handle, context, goal.intent)
            except Exception as exception:
                return self._abort(goal_handle, goal, str(exception), context)
        return self._execute_in_context(goal_handle, goal, context)

    async def _execute_pose_save(self, goal_handle, context):
        """Yield the executor while the coordinator collects fresh observations."""
        goal = goal_handle.request
        request = None
        timer = None
        completed = Future()
        last_detail = None
        is_landmark = goal.intent.operation == NavigationSetupIntent.OPERATION_ADD_VISIBLE_TAG_LANDMARK
        kind = "landmark" if is_landmark else "waypoint"

        def poll():
            nonlocal last_detail
            with self._lock:
                if completed.done():
                    return
                try:
                    if self._shutdown.is_set():
                        raise RuntimeError(f"Application shutdown interrupted {kind} saving")
                    if (goal_handle.is_cancel_requested
                            or self._admission.cancel_requested(id(goal))):
                        completed.set_result(None)
                        return
                    try:
                        snapshot = self.coordinator.poll_pose_save(request)
                    except NavigationObservationPending as exception:
                        detail = f"Collecting {kind}: {exception}"
                        if detail != last_detail:
                            state = self._state(
                                self.coordinator.snapshot(context), int(goal.intent.operation),
                                NavigationSetupState.STATE_RUNNING, detail, request.request_id,
                            )
                            self._state_publisher.publish(state)
                            feedback = ExecuteNavigationSetup.Feedback(state=state)
                            goal_handle.publish_feedback(feedback)
                            last_detail = detail
                    else:
                        completed.set_result(snapshot)
                except Exception as exception:
                    completed.set_exception(exception)

        try:
            if is_landmark:
                request = self.coordinator.begin_landmark_save(
                    context, goal.intent.map_name, int(goal.intent.tag_id),
                )
            else:
                request = self.coordinator.begin_waypoint_save(
                    context, goal.intent.map_name, goal.intent.waypoint_name,
                )
            with self._lock:
                self._pose_waiters.add(completed)
            timer = self.node.create_timer(
                0.1, poll, clock=Clock(clock_type=ClockType.STEADY_TIME),
                callback_group=self._callback_group,
            )
            poll()
            snapshot = await completed
            if snapshot is None:
                state = self._state(
                    self.coordinator.snapshot(context), int(goal.intent.operation),
                    NavigationSetupState.STATE_CANCELLED, f"{kind.capitalize()} saving cancelled",
                    request.request_id,
                )
                self._state_publisher.publish(state)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return ExecuteNavigationSetup.Result(state=state)
            return self._succeed(goal_handle, self._state(
                snapshot, int(goal.intent.operation), NavigationSetupState.STATE_SUCCEEDED,
                f"{kind.capitalize()} saved", request.request_id,
            ))
        except Exception as exception:
            return self._abort(
                goal_handle, goal, str(exception), context,
                request.request_id if request is not None else "",
            )
        finally:
            if timer is not None:
                self.node.destroy_timer(timer)
            with self._lock:
                self._pose_waiters.discard(completed)
            if request is not None:
                self.coordinator.finish_pose_save(request)

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

    async def _execute_runtime(self, goal_handle, context, intent):
        operation_code = int(intent.operation)
        with self._lock:
            self._runtime_submissions += 1
        try:
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
            )
            with self._lock:
                self._executions[operation.request_id] = execution
                early_state = self._early_states.pop(operation.request_id, None)
                if early_state is not None:
                    self._update_execution(execution, early_state)
        finally:
            with self._lock:
                self._runtime_submissions -= 1
                if not self._runtime_submissions:
                    self._early_states.clear()
        try:
            if self._shutdown.is_set():
                if not execution.finished.done():
                    raise RuntimeError("Application shutdown interrupted navigation setup; stop unconfirmed")
            if (goal_handle.is_cancel_requested
                    or self._admission.cancel_requested(id(goal_handle.request))):
                self._request_execution_cancellation(execution)
            state = await execution.finished
            return self._runtime_result(goal_handle, state)
        except Exception as exception:
            return self._abort(goal_handle, goal_handle.request, str(exception), context, operation.request_id)
        finally:
            with self._lock:
                self._executions.pop(execution.request_id, None)

    def _runtime_result(self, goal_handle, state):
        result = ExecuteNavigationSetup.Result(state=state)
        if state.state == NavigationSetupState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif (
            state.state in {
                NavigationSetupState.STATE_CANCELLED,
                NavigationSetupState.STATE_STOP_CONFIRMED,
            }
            and goal_handle.is_cancel_requested
        ):
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _update_execution(self, execution, state):
        with self._lock:
            if execution.finished.done():
                return
            if state.state in _TERMINAL_STATES:
                execution.finished.set_result(state)
        try:
            execution.goal_handle.publish_feedback(ExecuteNavigationSetup.Feedback(state=state))
        except Exception as exception:
            self.node.get_logger().error(f"Navigation setup feedback publication failed: {exception}")

    def _receive_status(self, status: NavigationSetupStatus) -> None:
        state = self._state(
            status.snapshot,
            status.operation_code,
            self._public_state(status.state),
            status.detail,
            status.request_id,
        )
        state.execution_instance_id = status.execution_instance_id
        state.status_sequence = status.status_sequence
        with self._lock:
            execution = self._executions.get(status.request_id)
            if execution is None:
                # Only submission can emit before the returned request is registered.
                # Completed requests must not accumulate in this handoff buffer.
                if status.request_id and self._runtime_submissions:
                    previous = self._early_states.get(status.request_id)
                    if previous is None or previous.state not in _TERMINAL_STATES:
                        self._early_states[status.request_id] = state
            else:
                self._update_execution(execution, state)
        try:
            self._state_publisher.publish(state)
        except Exception as exception:
            self.node.get_logger().error(f"Navigation setup state publication failed: {exception}")
        if not status.request_id:
            return
        self.node.get_logger().info(
            "Navigation setup request "
            f"{status.request_id}: {status.state.value}: "
            f"{status.detail}"
        )

    def _close(self, request, response):
        try:
            context = self.coordinator.context(
                request.context_id,
                request.client_id,
            )
            self._admission.cancel_context(context.context_id)
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

    def _abort(self, goal_handle, goal, detail, context=None, request_id="", *, cancelled=False):
        state_code = (NavigationSetupState.STATE_CANCELLED if cancelled else NavigationSetupState.STATE_FAILED)
        snapshot = None
        if context is not None:
            try:
                snapshot = self.coordinator.current_snapshot(context)
            except (LookupError, ValueError):
                pass  # Closing setup can invalidate a waiting action's context.
        if snapshot is not None:
            state = self._state(
                snapshot,
                int(goal.intent.operation),
                state_code,
                detail,
                request_id,
            )
            self._state_publisher.publish(state)
            result = ExecuteNavigationSetup.Result()
            result.state = state
            if cancelled and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        state = NavigationSetupState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.request_id = request_id or uuid4().hex
        state.client_id = goal.client_id.strip()
        state.context_id = goal.context_id.strip()
        state.revision = context.revision if context is not None else 0
        state.operation = int(goal.intent.operation)
        state.state = state_code
        state.detail = detail
        state.runtime_error = "Runtime status unavailable"
        self._state_publisher.publish(state)
        result = ExecuteNavigationSetup.Result()
        result.state = state
        if cancelled and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
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
        message.detail = (
            f"{detail}; {snapshot.runtime_error}" if snapshot.runtime_error else detail
        )
        message.runtime_error = snapshot.runtime_error
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
            CommandControllerState.UNKNOWN: NavigationSetupState.STATE_UNKNOWN,
            CommandControllerState.STOP_REQUESTED: NavigationSetupState.STATE_STOP_REQUESTED,
            CommandControllerState.STOP_CONFIRMED: NavigationSetupState.STATE_STOP_CONFIRMED,
            CommandControllerState.STOP_UNCONFIRMED: NavigationSetupState.STATE_STOP_UNCONFIRMED,
        }[state]

    @staticmethod
    def _mode_code(mode: str) -> int:
        if mode == MODE_MAPPING:
            return NavigationSetupState.MODE_MAPPING
        if mode == MODE_LOCALIZATION:
            return NavigationSetupState.MODE_LOCALIZATION
        return NavigationSetupState.MODE_NONE

    def request_shutdown(self) -> None:
        """Release action waits without destroying resources used by callbacks."""
        self._shutdown.set()
        self._admission.close()
        with self._lock:
            for waiter in self._pose_waiters:
                if not waiter.done():
                    waiter.set_exception(RuntimeError("Application shutdown interrupted navigation pose saving"))
            executions = tuple(self._executions.values())
        for execution in executions:
            with self._lock:
                if not execution.finished.done():
                    execution.finished.set_exception(RuntimeError(
                        "Application shutdown interrupted navigation setup; stop unconfirmed"
                    ))

    def close(self) -> None:
        """Detach transport resources from the coordinator."""
        release_resources(self, {
            "status_listener": lambda: self.coordinator.remove_status_listener(self._receive_status),
            "runtime_timer": lambda: self.node.destroy_timer(self._runtime_timer),
            "runtime_subscription": lambda: self.node.destroy_subscription(self._runtime_subscription),
            "action_server": self._action_server.destroy,
        })


__all__ = ["NavigationSetupApi"]
