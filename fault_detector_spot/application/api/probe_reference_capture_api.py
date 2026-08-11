"""Expose synchronized probe reference capture as one ROS action."""

from uuid import uuid4

from fault_detector_msgs.action import CaptureProbeReferenceViews
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureSpec,
    ReferenceCaptureCancelled,
    ReferenceCapturePhase,
)
from fault_detector_spot.application.setup.setup_context import (
    validate_context_id,
)
from fault_detector_spot.inspection.setup.reference_camera_registry import (
    validate_reference_camera_slots,
)


class ProbeReferenceCaptureApi:
    """Own ROS transport for server-owned reference dataset capture."""

    def __init__(
        self,
        node,
        probe_setup_coordinator,
        capture_coordinator,
        state_publisher,
        state_adapter,
    ):
        self.node = node
        self.probe_setup_coordinator = probe_setup_coordinator
        self.capture_coordinator = capture_coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            node,
            CaptureProbeReferenceViews,
            "fault_detector/application/capture_probe_reference_views",
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )

    def _accept(self, goal_request):
        try:
            required_client_id(goal_request.client_id)
            validate_context_id(goal_request.context_id)
            validate_reference_camera_slots(
                tuple(goal_request.reference_camera_ids)
            )
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        request_id = uuid4().hex
        context = None
        try:
            context = self.probe_setup_coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            spec = ProbeReferenceCaptureSpec(
                reference_camera_ids=tuple(
                    goal.reference_camera_ids
                ),
                replace_existing=bool(goal.replace_existing),
            )
            snapshot = self.capture_coordinator.run(
                context,
                request_id,
                spec,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                state_changed=lambda phase, attempt, current: (
                    self._publish_feedback(
                        goal_handle,
                        phase,
                        attempt,
                        current,
                        request_id,
                    )
                ),
            )
        except ReferenceCaptureCancelled as exception:
            state = self._terminal_state(
                goal,
                context,
                request_id,
                ProbeSetupState.STATE_CANCELLED,
                str(exception),
            )
            goal_handle.canceled()
            return self._result(state)
        except Exception as exception:
            state = self._terminal_state(
                goal,
                context,
                request_id,
                ProbeSetupState.STATE_FAILED,
                str(exception),
            )
            goal_handle.abort()
            return self._result(state)

        state = self.state_adapter.message(
            snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            ProbeSetupState.STATE_SUCCEEDED,
            "Reference dataset capture completed",
            request_id=request_id,
        )
        self.state_publisher.publish(state)
        goal_handle.succeed()
        return self._result(state)

    def _publish_feedback(
        self,
        goal_handle,
        phase,
        attempt,
        snapshot,
        request_id,
    ):
        state = self.state_adapter.message(
            snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            ProbeSetupState.STATE_RUNNING,
            phase.value.replace("_", " "),
            request_id=request_id,
        )
        self.state_publisher.publish(state)
        feedback = CaptureProbeReferenceViews.Feedback()
        feedback.phase = self._phase_code(phase)
        feedback.attempt = int(attempt)
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    def _terminal_state(
        self,
        goal,
        context,
        request_id,
        state_code,
        detail,
    ):
        if (
            context is not None
            and self.probe_setup_coordinator.setup_coordinator.is_current(
                context
            )
        ):
            snapshot = self.probe_setup_coordinator.snapshot(context)
            state = self.state_adapter.message(
                snapshot,
                ProbeSetupIntent.OPERATION_UNSPECIFIED,
                state_code,
                detail,
                request_id=request_id,
            )
        else:
            state = ProbeSetupState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.request_id = request_id
            state.client_id = goal.client_id.strip()
            state.context_id = goal.context_id.strip()
            state.state = state_code
            state.detail = detail
            state.validation_error = detail
        self.state_publisher.publish(state)
        return state

    @staticmethod
    def _result(state):
        result = CaptureProbeReferenceViews.Result()
        result.state = state
        return result

    @staticmethod
    def _phase_code(phase):
        values = {
            ReferenceCapturePhase.WAITING_FOR_INPUTS: (
                CaptureProbeReferenceViews.Feedback.PHASE_WAITING_FOR_INPUTS
            ),
            ReferenceCapturePhase.VALIDATING: (
                CaptureProbeReferenceViews.Feedback.PHASE_VALIDATING
            ),
            ReferenceCapturePhase.SAVING: (
                CaptureProbeReferenceViews.Feedback.PHASE_SAVING
            ),
            ReferenceCapturePhase.COMPLETE: (
                CaptureProbeReferenceViews.Feedback.PHASE_COMPLETE
            ),
        }
        return values[phase]

    def close(self):
        """Stop capture work and destroy its action transport."""
        self.capture_coordinator.close()
        self._action_server.destroy()


__all__ = ["ProbeReferenceCaptureApi"]
