"""Expose probe setup motion through one typed ROS action."""

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

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
    ProbeSetupMotionStatus,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeAlignmentOrientationMode,
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
    finished: Event = field(default_factory=Event)
    state: object = None
    cancellation_requested: bool = False


class ProbeSetupMotionApi:
    """Own the asynchronous lifecycle of probe setup motion requests."""

    def __init__(
        self,
        node,
        coordinator: ProbeSetupCoordinator,
        state_publisher,
        state_adapter: ProbeSetupStateAdapter,
    ):
        self.node = node
        self.coordinator = coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self._lock = RLock()
        self._executions = {}
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
        try:
            required_client_id(goal_request.client_id)
            self._motion_request(goal_request.intent)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
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
            self.coordinator.submit_motion(operation)
        except Exception as exception:
            with self._lock:
                self._executions.pop(operation.request_id, None)
            return self._abort(goal_handle, goal, str(exception))
        while not execution.finished.wait(0.05):
            if (
                goal_handle.is_cancel_requested
                and not execution.cancellation_requested
            ):
                execution.cancellation_requested = True
                try:
                    self.coordinator.cancel_motion(
                        context,
                        operation.request_id,
                    )
                except LookupError:
                    pass
        with self._lock:
            self._executions.pop(operation.request_id, None)
        result = ExecuteProbeSetupMotion.Result()
        result.state = execution.state
        if execution.state.state == ProbeSetupState.STATE_SUCCEEDED:
            goal_handle.succeed()
        elif execution.state.state == ProbeSetupState.STATE_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _receive_status(self, status: ProbeSetupMotionStatus) -> None:
        if status.motion.kind is ProbeMotionKind.ADJUST_PROBE_DISTANCE:
            return
        state = self.state_adapter.message(
            status.snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            self._public_state(status.state),
            status.detail,
            request_id=status.request_id,
            motion_operation=self._motion_operation(status.motion.kind),
        )
        self.state_publisher.publish(state)
        with self._lock:
            execution = self._executions.get(status.request_id)
        if execution is None:
            return
        execution.state = state
        feedback = ExecuteProbeSetupMotion.Feedback()
        feedback.state = state
        execution.goal_handle.publish_feedback(feedback)
        if status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }:
            execution.finished.set()

    def _abort(self, goal_handle, goal, detail):
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            snapshot = self.coordinator.snapshot(context)
            state = self.state_adapter.message(
                snapshot,
                ProbeSetupIntent.OPERATION_UNSPECIFIED,
                ProbeSetupState.STATE_FAILED,
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
            state.state = ProbeSetupState.STATE_FAILED
            state.detail = detail
            state.validation_error = detail
        self.state_publisher.publish(state)
        result = ExecuteProbeSetupMotion.Result()
        result.state = state
        goal_handle.abort()
        return result

    @staticmethod
    def _motion_request(intent):
        kinds = {
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
        }
        frames = {
            ProbeSetupMotionIntent.FRAME_SENSOR: ProbeMotionFrame.SENSOR,
            ProbeSetupMotionIntent.FRAME_HAND: ProbeMotionFrame.HAND,
            ProbeSetupMotionIntent.FRAME_TAG: ProbeMotionFrame.TAG,
            ProbeSetupMotionIntent.FRAME_BODY: ProbeMotionFrame.BODY,
            ProbeSetupMotionIntent.FRAME_MAP: ProbeMotionFrame.MAP,
        }
        modes = {
            ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_TAG: (
                ProbeAlignmentOrientationMode.TAG
            ),
            ProbeSetupMotionIntent.ALIGNMENT_ORIENTATION_CALCULATED_SURFACE: (
                ProbeAlignmentOrientationMode.CALCULATED_SURFACE
            ),
        }
        try:
            kind = kinds[int(intent.operation)]
            frame = frames[int(intent.frame)]
            mode = modes[int(intent.alignment_orientation_mode)]
        except KeyError as exception:
            raise ValueError(
                "Unsupported probe setup motion intent"
            ) from exception
        calculated_orientation = None
        if bool(intent.has_calculated_surface_orientation):
            value = intent.calculated_surface_orientation_object
            calculated_orientation = QuaternionData(
                x=float(value.x),
                y=float(value.y),
                z=float(value.z),
                w=float(value.w),
            )
        request = ProbeMotionRequest(
            kind=kind,
            frame=frame,
            translation=Vector3Data(
                x=float(intent.translation.x),
                y=float(intent.translation.y),
                z=float(intent.translation.z),
            ),
            pitch_rad=float(intent.pitch_rad),
            yaw_rad=float(intent.yaw_rad),
            position_tolerance_m=float(intent.position_tolerance_m),
            orientation_tolerance_rad=float(
                intent.orientation_tolerance_rad
            ),
            alignment_orientation_mode=mode,
            orientation_only=bool(intent.orientation_only),
            calculated_surface_orientation_object=(
                calculated_orientation
            ),
        )
        request.validate()
        return request

    @staticmethod
    def _motion_operation(kind):
        return {
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
        }[state]

    def close(self) -> None:
        """Destroy the motion action and detach status observation."""
        self.coordinator.remove_motion_status_listener(self._receive_status)
        self._action_server.destroy()
        with self._lock:
            self._executions.clear()


__all__ = ["ProbeSetupMotionApi"]
