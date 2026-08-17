"""Expose server-owned probe surface verification through one ROS action."""

from uuid import uuid4

from fault_detector_msgs.action import ExecuteProbeSurfaceVerification
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.setup.setup_context import (
    validate_context_id,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    SurfaceVerificationState,
)


class ProbeSurfaceVerificationApi:
    """Own ROS transport for the server-side verification workflow."""

    def __init__(
        self,
        node,
        coordinator,
        state_publisher,
        state_adapter,
    ):
        self.node = node
        self.coordinator = coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            node,
            ExecuteProbeSurfaceVerification,
            "fault_detector/application/execute_probe_surface_verification",
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group,
        )

    def _accept(self, goal_request):
        try:
            required_client_id(goal_request.client_id)
            validate_context_id(goal_request.context_id)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        request_id = str(uuid4())
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            snapshot = self.coordinator.run_surface_verification(
                context,
                request_id,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                state_changed=lambda value: self._publish_feedback(
                    goal_handle,
                    value,
                ),
            )
            state = self._state_message(
                snapshot,
                cancelled=goal_handle.is_cancel_requested,
            )
        except Exception as exception:
            return self._abort(
                goal_handle,
                goal,
                request_id,
                str(exception),
            )

        self.state_publisher.publish(state)
        result = ExecuteProbeSurfaceVerification.Result()
        result.state = state
        verification = snapshot.surface_verification
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif (
            verification is not None
            and verification.state is SurfaceVerificationState.CONVERGED
        ):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _publish_feedback(self, goal_handle, snapshot):
        state = self._state_message(
            snapshot,
            cancelled=goal_handle.is_cancel_requested,
        )
        self.state_publisher.publish(state)
        feedback = ExecuteProbeSurfaceVerification.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    def _state_message(self, snapshot, cancelled=False):
        verification = snapshot.surface_verification
        if verification is None:
            state_code = ProbeSetupState.STATE_FAILED
            detail = "Surface verification state is unavailable"
            request_id = ""
        else:
            request_id = verification.request_id
            detail = verification.detail
            if cancelled and not verification.active:
                state_code = ProbeSetupState.STATE_CANCELLED
            elif verification.state is SurfaceVerificationState.CONVERGED:
                state_code = ProbeSetupState.STATE_SUCCEEDED
            elif verification.state is SurfaceVerificationState.CANCELLED:
                state_code = ProbeSetupState.STATE_CANCELLED
            elif verification.state in {
                SurfaceVerificationState.FAILED,
                SurfaceVerificationState.RECOVERY_REQUIRED,
            }:
                state_code = ProbeSetupState.STATE_FAILED
            else:
                state_code = ProbeSetupState.STATE_RUNNING
        return self.state_adapter.message(
            snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            state_code,
            detail,
            request_id=request_id,
        )

    def _abort(self, goal_handle, goal, request_id, detail):
        snapshot = None
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            current = self.coordinator.snapshot(context)
            verification = current.surface_verification
            if (
                verification is not None
                and verification.active
                and verification.request_id == request_id
            ):
                snapshot = self.coordinator.abort_surface_verification(
                    context,
                    request_id,
                    detail,
                )
            else:
                snapshot = current
        except Exception:
            snapshot = None

        if snapshot is not None:
            state = self.state_adapter.message(
                snapshot,
                ProbeSetupIntent.OPERATION_UNSPECIFIED,
                ProbeSetupState.STATE_FAILED,
                detail,
                request_id=request_id,
            )
        else:
            state = ProbeSetupState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.request_id = request_id
            state.client_id = goal.client_id.strip()
            state.context_id = goal.context_id.strip()
            state.state = ProbeSetupState.STATE_FAILED
            state.detail = detail
            state.validation_error = detail
        self.state_publisher.publish(state)
        result = ExecuteProbeSurfaceVerification.Result()
        result.state = state
        goal_handle.abort()
        return result

    def close(self):
        """Destroy the action transport."""
        self._action_server.destroy()


__all__ = ["ProbeSurfaceVerificationApi"]

