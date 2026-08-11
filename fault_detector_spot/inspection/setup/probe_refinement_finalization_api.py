"""Expose probe refinement finalization as one typed ROS action."""

from uuid import uuid4

from fault_detector_msgs.action import FinalizeProbeRefinement
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.setup.setup_context import (
    validate_context_id,
)
from fault_detector_spot.inspection.setup.probe_refinement_finalization import (
    FinalizationPhase,
    ProbeRefinementFinalizationRunner,
    ProbeRefinementFinalizationSpec,
)


class ProbeRefinementFinalizationApi:
    """Own ROS transport for save and mandatory retraction."""

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
        self.runner = ProbeRefinementFinalizationRunner(coordinator)
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            node,
            FinalizeProbeRefinement,
            "fault_detector/application/finalize_probe_refinement",
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
        if goal_request.mode not in {
            FinalizeProbeRefinement.Goal.MODE_SAVE_AND_RETRACT,
            FinalizeProbeRefinement.Goal.MODE_RETRACT_WITHOUT_SAVING,
        }:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        request_id = str(uuid4())
        spec = ProbeRefinementFinalizationSpec(
            save_requested=(
                goal.mode
                == FinalizeProbeRefinement.Goal.MODE_SAVE_AND_RETRACT
            ),
            probe_point_id=goal.probe_point_id,
            probe_point_display_name=goal.probe_point_display_name,
            position_tolerance_m=goal.position_tolerance_m,
            orientation_tolerance_rad=goal.orientation_tolerance_rad,
            measurement_duration_sec=goal.measurement_duration_sec,
        )
        saved = False
        phase = FinalizationPhase.RECOVERY_REQUIRED
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            snapshot, saved, phase = self.runner.run(
                context,
                request_id,
                spec,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                state_changed=lambda value, was_saved, current: (
                    self._publish_feedback(
                        goal_handle,
                        value,
                        was_saved,
                        current,
                        request_id,
                    )
                ),
            )
        except Exception as exception:
            snapshot = self._fail_active(
                goal,
                request_id,
                str(exception),
            )
            return self._abort(
                goal_handle,
                goal,
                request_id,
                saved,
                snapshot,
                str(exception),
            )

        cancelled = bool(goal_handle.is_cancel_requested)
        state_code = self._terminal_state_code(phase, cancelled)
        detail = self._terminal_detail(
            snapshot,
            phase,
            cancelled,
        )
        state = self._state_message(
            snapshot,
            state_code,
            detail,
            request_id,
        )
        self.state_publisher.publish(state)
        result = FinalizeProbeRefinement.Result()
        result.probe_point_saved = saved
        result.state = state
        if phase is FinalizationPhase.COMPLETE:
            goal_handle.succeed()
        elif cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _publish_feedback(
        self,
        goal_handle,
        phase,
        saved,
        snapshot,
        request_id,
    ):
        cancelled = bool(goal_handle.is_cancel_requested)
        if phase is FinalizationPhase.RECOVERY_REQUIRED:
            state_code = self._terminal_state_code(phase, cancelled)
            detail = self._terminal_detail(
                snapshot,
                phase,
                cancelled,
            )
        else:
            state_code = ProbeSetupState.STATE_RUNNING
            detail = phase.value.replace("_", " ")
        state = self._state_message(
            snapshot,
            state_code,
            detail,
            request_id,
        )
        self.state_publisher.publish(state)
        feedback = FinalizeProbeRefinement.Feedback()
        feedback.phase = self._phase_code(phase)
        feedback.probe_point_saved = saved
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    def _fail_active(self, goal, request_id, detail):
        try:
            context = self.coordinator.context(
                goal.context_id,
                goal.client_id,
            )
            return self.coordinator.fail_finalization(
                context,
                request_id,
                detail,
            )
        except Exception:
            try:
                context = self.coordinator.context(
                    goal.context_id,
                    goal.client_id,
                )
                return self.coordinator.snapshot(context)
            except Exception:
                return None

    def _abort(
        self,
        goal_handle,
        goal,
        request_id,
        saved,
        snapshot,
        detail,
    ):
        cancelled = bool(goal_handle.is_cancel_requested)
        state_code = (
            ProbeSetupState.STATE_CANCELLED
            if cancelled
            else ProbeSetupState.STATE_FAILED
        )
        if snapshot is not None:
            state = self._state_message(
                snapshot,
                state_code,
                detail,
                request_id,
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
        result = FinalizeProbeRefinement.Result()
        result.probe_point_saved = saved
        result.state = state
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _state_message(self, snapshot, state_code, detail, request_id):
        return self.state_adapter.message(
            snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            state_code,
            detail,
            request_id=request_id,
        )

    @staticmethod
    def _terminal_state_code(phase, cancelled):
        if phase is FinalizationPhase.COMPLETE:
            return ProbeSetupState.STATE_SUCCEEDED
        if cancelled:
            return ProbeSetupState.STATE_CANCELLED
        return ProbeSetupState.STATE_FAILED

    @staticmethod
    def _terminal_detail(snapshot, phase, cancelled):
        if phase is FinalizationPhase.COMPLETE:
            return "Probe refinement finalized"
        refinement = getattr(snapshot, "refinement", None)
        if refinement is not None:
            detail = refinement.recovery_message.strip()
            if detail:
                return detail
        if cancelled:
            return (
                "Probe refinement finalization cancelled; "
                "retraction is required"
            )
        return "Probe refinement finalization requires recovery"

    @staticmethod
    def _phase_code(phase):
        values = {
            FinalizationPhase.APPROVING: (
                FinalizeProbeRefinement.Feedback.PHASE_APPROVING
            ),
            FinalizationPhase.SAVING: (
                FinalizeProbeRefinement.Feedback.PHASE_SAVING
            ),
            FinalizationPhase.RETRACTING_TO_ALIGNED: (
                FinalizeProbeRefinement.Feedback.PHASE_RETRACTING_TO_ALIGNED
            ),
            FinalizationPhase.RETRACTING_TO_SAFE: (
                FinalizeProbeRefinement.Feedback.PHASE_RETRACTING_TO_SAFE
            ),
            FinalizationPhase.COMPLETE: (
                FinalizeProbeRefinement.Feedback.PHASE_COMPLETE
            ),
            FinalizationPhase.RECOVERY_REQUIRED: (
                FinalizeProbeRefinement.Feedback.PHASE_RECOVERY_REQUIRED
            ),
        }
        return values[phase]

    def close(self):
        """Destroy finalization transport and detach its runner."""
        self.runner.close()
        self._action_server.destroy()


__all__ = ["ProbeRefinementFinalizationApi"]
