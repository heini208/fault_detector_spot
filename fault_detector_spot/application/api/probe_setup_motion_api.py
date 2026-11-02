"""Expose probe setup motion through one typed ROS action."""

from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.api.shutdown_callbacks import (
    ShutdownCallbackActivity, tracked_shutdown_callback, release_resources,
)

from dataclasses import dataclass, field
from threading import Event, RLock
from uuid import uuid4

from fault_detector_msgs.action import ExecuteProbeSetupMotion
from fault_detector_msgs.msg import (
    ProbeSetupIntent,
    ProbeSetupMotionIntent,
    ProbeSetupState,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.shared.geometry.models import (
    Vector3Data,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
    ProbeSetupMotionStatus,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionFrame,
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import (
    ProbeSetupStateAdapter,
)


@dataclass
class _ProbeMotionExecution:
    goal_handle: object
    request_id: str
    context: object
    finished: Future = field(default_factory=Future)
    cancellation_sent: bool = False
    submitted: bool = False


class ProbeSetupMotionApi:
    """Own the asynchronous lifecycle of probe setup motion requests."""

    def __init__(
        self,
        node,
        coordinator: ProbeSetupCoordinator,
        state_publisher,
        state_adapter: ProbeSetupStateAdapter,
        admission=None,
    ):
        self.node = node
        self.coordinator = coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self._lock = RLock()
        self._executions = {}
        self._admission = admission or OperationAdmission(limit=1)
        self._shutdown = Event()
        self._shutdown_callbacks = ShutdownCallbackActivity()
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            node,
            ExecuteProbeSetupMotion,
            "fault_detector/application/execute_probe_setup_motion",
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )
        coordinator.add_motion_status_listener(self._receive_status)

    def _accept(self, goal_request):
        if self._shutdown.is_set():
            return GoalResponse.REJECT
        try:
            required_client_id(goal_request.client_id)
            self._motion_request(goal_request.intent)
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
            if (not execution.submitted or execution.cancellation_sent
                    or (execution.finished.done() and execution.finished.exception() is None)):
                return
            execution.cancellation_sent = True
        try:
            self.coordinator.cancel_motion(execution.context, execution.request_id)
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
                goal_handle, goal, "Probe motion cancelled before dispatch", cancelled=True,
            )
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            motion = self._motion_request(goal.intent)
            operation = self.coordinator.prepare_motion(context, motion)
        except Exception as exception:
            return self._abort(goal_handle, goal, str(exception))
        execution = _ProbeMotionExecution(
            goal_handle=goal_handle,
            request_id=operation.request_id,
            context=context,
        )
        with self._lock:
            self._executions[operation.request_id] = execution
        try:
            try:
                if not execution.finished.done():
                    if (goal_handle.is_cancel_requested
                            or self._admission.cancel_requested(id(goal))):
                        execution.cancellation_sent = True
                        self.coordinator.cancel_motion(context, operation.request_id)
                    else:
                        self.coordinator.submit_motion(operation)
            except Exception:
                if not execution.finished.done():
                    raise
            with self._lock:
                execution.submitted = True
            if self._shutdown.is_set():
                self._request_execution_cancellation(execution)
                if not execution.finished.done():
                    raise RuntimeError("Application shutdown interrupted probe motion; stop unconfirmed")
            if (goal_handle.is_cancel_requested
                    or self._admission.cancel_requested(id(goal))):
                self._request_execution_cancellation(execution)
            state = await execution.finished
        except Exception as exception:
            return self._abort(goal_handle, goal, str(exception))
        finally:
            with self._lock:
                self._executions.pop(operation.request_id, None)
        result = ExecuteProbeSetupMotion.Result(state=state)
        if state.state == ProbeSetupState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif state.state in {
            ProbeSetupState.STATE_CANCELLED,
            ProbeSetupState.STATE_STOP_CONFIRMED,
        } and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _receive_status(self, status: ProbeSetupMotionStatus) -> None:
        state = self.state_adapter.message(
            status.snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            self._public_state(status.state),
            status.detail,
            request_id=status.request_id,
            motion_operation=self._motion_operation(status.motion.kind),
        )
        state.execution_instance_id = status.execution_instance_id
        state.status_sequence = status.status_sequence
        with self._lock:
            execution = self._executions.get(status.request_id)
            if execution is not None and not execution.finished.done():
                if status.state in {
                    CommandControllerState.SUCCEEDED,
                    CommandControllerState.FAILED,
                    CommandControllerState.CANCELLED,
                    CommandControllerState.STOP_CONFIRMED,
                }:
                    execution.finished.set_result(state)
            else:
                execution = None
        try:
            self.state_publisher.publish(state)
        except Exception as exception:
            self.node.get_logger().error(f"Probe motion state publication failed: {exception}")
        if execution is not None:
            feedback = ExecuteProbeSetupMotion.Feedback(state=state)
            try:
                execution.goal_handle.publish_feedback(feedback)
            except Exception as exception:
                self.node.get_logger().error(f"Probe motion feedback publication failed: {exception}")

    def _abort(self, goal_handle, goal, detail, *, cancelled=False):
        state_code = (ProbeSetupState.STATE_CANCELLED if cancelled else ProbeSetupState.STATE_FAILED)
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            snapshot = self.coordinator.snapshot(context)
            state = self.state_adapter.message(
                snapshot,
                ProbeSetupIntent.OPERATION_UNSPECIFIED,
                state_code,
                detail,
                motion_operation=int(goal.intent.operation),
            )
        except Exception:
            state = ProbeSetupState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.request_id = uuid4().hex
            state.client_id = goal.client_id.strip()
            state.context_id = goal.context_id.strip()
            state.motion_operation = int(goal.intent.operation)
            state.state = state_code
            state.detail = detail
            state.validation_error = detail
        self.state_publisher.publish(state)
        result = ExecuteProbeSetupMotion.Result()
        result.state = state
        if cancelled and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _motion_request(intent):
        kinds = {
            ProbeSetupMotionIntent.OPERATION_ADJUST_PATHING_POSE: ProbeMotionKind.ADJUST_PATHING_POSE,
            ProbeSetupMotionIntent.OPERATION_MOVE_PATHING_POINT: ProbeMotionKind.MOVE_PATHING_POINT,
            ProbeSetupMotionIntent.OPERATION_MOVE_PRE_APPROACH_PATH: ProbeMotionKind.MOVE_PRE_APPROACH_PATH,
            ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH: (
                ProbeMotionKind.MOVE_SAFE_APPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH: (
                ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_ADJUST_SAFE_APPROACH: (
                ProbeMotionKind.ADJUST_SAFE_APPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH: (
                ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH
            ),
            ProbeSetupMotionIntent.OPERATION_ORIENT_TO_SURFACE: (
                ProbeMotionKind.ORIENT_TO_SURFACE
            ),
            ProbeSetupMotionIntent.OPERATION_ORIENT_SENSOR_TIP_TO_SURFACE: (
                ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE
            ),
            ProbeSetupMotionIntent.OPERATION_ORIENT_TO_TAG: (
                ProbeMotionKind.ORIENT_TO_TAG
            ),
        }
        frames = {
            ProbeSetupMotionIntent.FRAME_SENSOR: ProbeMotionFrame.SENSOR,
            ProbeSetupMotionIntent.FRAME_HAND: ProbeMotionFrame.HAND,
            ProbeSetupMotionIntent.FRAME_TAG: ProbeMotionFrame.TAG,
            ProbeSetupMotionIntent.FRAME_BODY: ProbeMotionFrame.BODY,
            ProbeSetupMotionIntent.FRAME_MAP: ProbeMotionFrame.MAP,
        }
        try:
            kind = kinds[int(intent.operation)]
            frame = frames[int(intent.frame)]
        except KeyError as exception:
            raise ValueError(
                "Unsupported probe setup motion intent"
            ) from exception
        request = ProbeMotionRequest(
            kind=kind,
            path_stage=intent.path_stage,
            arm_speed_scale=float(intent.arm_speed_scale),
            pathing_point_index=int(intent.pathing_point_index),
            frame=frame,
            translation=Vector3Data(
                x=float(intent.translation.x),
                y=float(intent.translation.y),
                z=float(intent.translation.z),
            ),
            pitch_rad=float(intent.pitch_rad),
            yaw_rad=float(intent.yaw_rad),
            position_tolerance_m=float(intent.position_tolerance_m),
            precision_mode=intent.precision_mode,
            orientation_tolerance_rad=float(
                intent.orientation_tolerance_rad
            ),
        )
        request.validate()
        return request

    @staticmethod
    def _motion_operation(kind):
        return {
            ProbeMotionKind.ADJUST_PATHING_POSE: ProbeSetupMotionIntent.OPERATION_ADJUST_PATHING_POSE,
            ProbeMotionKind.MOVE_PATHING_POINT: ProbeSetupMotionIntent.OPERATION_MOVE_PATHING_POINT,
            ProbeMotionKind.MOVE_PRE_APPROACH_PATH: ProbeSetupMotionIntent.OPERATION_MOVE_PRE_APPROACH_PATH,
            ProbeMotionKind.MOVE_SAFE_APPROACH: (
                ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH
            ),
            ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH: (
                ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH
            ),
            ProbeMotionKind.ADJUST_SAFE_APPROACH: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_SAFE_APPROACH
            ),
            ProbeMotionKind.ADJUST_ALIGNED_PREAPPROACH: (
                ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH
            ),
            ProbeMotionKind.ORIENT_TO_SURFACE: (
                ProbeSetupMotionIntent.OPERATION_ORIENT_TO_SURFACE
            ),
            ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE: (
                ProbeSetupMotionIntent.OPERATION_ORIENT_SENSOR_TIP_TO_SURFACE
            ),
            ProbeMotionKind.ORIENT_TO_TAG: (
                ProbeSetupMotionIntent.OPERATION_ORIENT_TO_TAG
            ),
        }[kind]

    @staticmethod
    def _public_state(state):
        return {
            CommandControllerState.QUEUED: ProbeSetupState.STATE_QUEUED,
            CommandControllerState.DISPATCHED: ProbeSetupState.STATE_RUNNING,
            CommandControllerState.RUNNING: ProbeSetupState.STATE_RUNNING,
            CommandControllerState.SUCCEEDED: ProbeSetupState.STATE_SUCCEEDED,
            CommandControllerState.FAILED: ProbeSetupState.STATE_FAILED,
            CommandControllerState.CANCELLED: ProbeSetupState.STATE_CANCELLED,
            CommandControllerState.UNKNOWN: ProbeSetupState.STATE_UNKNOWN,
            CommandControllerState.STOP_REQUESTED: ProbeSetupState.STATE_STOP_REQUESTED,
            CommandControllerState.STOP_CONFIRMED: ProbeSetupState.STATE_STOP_CONFIRMED,
            CommandControllerState.STOP_UNCONFIRMED: ProbeSetupState.STATE_STOP_UNCONFIRMED,
        }[state]

    def request_shutdown(self) -> None:
        """Release action waits without destroying resources used by callbacks."""
        self._shutdown.set()
        self._admission.close()
        with self._lock:
            executions = tuple(self._executions.values())
        errors = []
        for execution in executions:
            try:
                self._request_execution_cancellation(execution)
            except Exception as exception:
                errors.append(str(exception))
            finally:
                with self._lock:
                    if not execution.finished.done():
                        execution.finished.set_exception(RuntimeError(
                            "Application shutdown interrupted probe motion; stop unconfirmed"
                        ))
        if errors:
            raise RuntimeError("; ".join(errors))

    def close(self) -> None:
        """Destroy the motion action and detach status observation."""
        def clear_executions():
            with self._lock:
                self._executions.clear()

        release_resources(self, {
            "motion_listener": lambda: self.coordinator.remove_motion_status_listener(self._receive_status),
            "action_server": self._action_server.destroy,
            "executions": clear_executions,
        })


__all__ = ["ProbeSetupMotionApi"]
