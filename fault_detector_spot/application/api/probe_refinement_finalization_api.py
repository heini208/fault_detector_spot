"""Expose probe refinement finalization as one typed ROS action."""

from fault_detector_spot.application.api.shutdown_callbacks import (
    ShutdownCallbackActivity, tracked_shutdown_callback, release_resources,
)

from threading import Event
from uuid import uuid4

from fault_detector_msgs.action import FinalizeProbeRefinement
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.api.workflow_worker import deliver_update, run_in_worker

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.setup.setup_context import (
    validate_context_id,
)
from fault_detector_spot.application.coordinators.probe_refinement_finalization_coordinator import (
    FinalizationPhase,
    ProbeRefinementFinalizationCoordinator,
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
        *,
        admission,
    ):
        self._shutdown = Event()
        self._shutdown_callbacks = ShutdownCallbackActivity()
        self._admission = admission
        self.node = node
        self.coordinator = coordinator
        self.state_publisher = state_publisher
        self.state_adapter = state_adapter
        self.runner = ProbeRefinementFinalizationCoordinator(coordinator)
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
        if self._shutdown.is_set():
            return GoalResponse.REJECT
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
        return (GoalResponse.ACCEPT if self._admission.reserve(
            (self, id(goal_request)), context_id=goal_request.context_id, lane="physical",
        ) else GoalResponse.REJECT)

    def _accept_cancel(self, goal_handle):
        self._admission.cancel((self, id(goal_handle.request)))
        return CancelResponse.ACCEPT

    @tracked_shutdown_callback
    async def _execute(self, goal_handle):
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
        snapshot = None
        key = (self, id(goal))

        def progress(value, was_saved, current):
            nonlocal saved, snapshot
            # Preserve committed data even if later feedback transport fails.
            saved = saved or was_saved
            snapshot = current
            self._publish_feedback(goal_handle, value, saved, current, request_id)

        def finalize(worker_cancelled):
            if worker_cancelled() or self._shutdown.is_set() or goal_handle.is_cancel_requested:
                raise RuntimeError("Probe finalization cancelled before execution")
            context = self.coordinator.context(goal.context_id, goal.client_id)

            def cancelled():
                return (worker_cancelled() or self._shutdown.is_set()
                        or goal_handle.is_cancel_requested
                        or self.coordinator.is_context_closing(context))

            return self.runner.run(
                context,
                request_id,
                spec,
                cancel_requested=cancelled,
                state_changed=progress,
            )

        try:
            if self._shutdown.is_set():
                raise RuntimeError("Application is shutting down")
            snapshot, saved, phase = await run_in_worker(self._admission, key, finalize)
        except Exception as exception:
            snapshot = self._fail_active(
                goal, request_id, str(exception),
            ) or snapshot
            saved = saved or bool(getattr(
                getattr(snapshot, "refinement", None), "saved", False,
            ))
            return self._abort(
                goal_handle,
                goal,
                request_id,
                saved,
                snapshot,
                str(exception),
            )
        finally:
            self._admission.release_pending(key)

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
        deliver_update(self.node, self.state_publisher.publish, state)
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
        deliver_update(self.node, self.state_publisher.publish, state)
        feedback = FinalizeProbeRefinement.Feedback()
        feedback.phase = self._phase_code(phase)
        feedback.probe_point_saved = saved
        feedback.state = state
        deliver_update(self.node, goal_handle.publish_feedback, feedback)

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
        if saved:
            # Even if the context or its last snapshot is no longer available,
            # failure after persistence must not hide the recovery obligation.
            state.refinement_recovery_required = True
            state.refinement_recovery_message = detail
        deliver_update(self.node, self.state_publisher.publish, state)
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

    def request_shutdown(self):
        """Stop setup work before destroying its action transport."""
        self._shutdown.set()
        self.runner.request_shutdown()

    def close(self):
        """Destroy finalization transport and detach its runner."""
        resources = {
            "runner": self.runner.close,
            "action_server": self._action_server.destroy,
        }
        release_resources(self, resources)


__all__ = ["ProbeRefinementFinalizationApi"]
