"""Expose synchronized probe reference capture as one ROS action."""

from fault_detector_spot.application.api.shutdown_callbacks import (
    ShutdownCallbackActivity, tracked_shutdown_callback, release_resources,
)

from threading import Event
from uuid import uuid4

from fault_detector_msgs.action import CaptureProbeReferenceViews
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from fault_detector_spot.application.api.workflow_worker import deliver_update, run_in_worker

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
        *,
        admission,
    ):
        self._shutdown = Event()
        self._shutdown_callbacks = ShutdownCallbackActivity()
        self._admission = admission
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
        if self._shutdown.is_set():
            return GoalResponse.REJECT
        try:
            required_client_id(goal_request.client_id)
            validate_context_id(goal_request.context_id)
            validate_reference_camera_slots(
                tuple(goal_request.reference_camera_ids)
            )
        except (TypeError, ValueError):
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
        request_id = uuid4().hex
        context = None
        key = (self, id(goal))

        def capture(worker_cancelled):
            nonlocal context
            if worker_cancelled() or self._shutdown.is_set() or goal_handle.is_cancel_requested:
                raise ReferenceCaptureCancelled("Reference capture cancelled before execution")
            context = self.probe_setup_coordinator.context(
                goal.context_id, goal.client_id,
            )
            spec = ProbeReferenceCaptureSpec(
                reference_camera_ids=tuple(goal.reference_camera_ids),
                replace_existing=bool(goal.replace_existing),
            )
            def cancelled():
                return (worker_cancelled() or self._shutdown.is_set()
                        or goal_handle.is_cancel_requested
                        or self.probe_setup_coordinator.is_context_closing(context))

            return self.capture_coordinator.run(
                context,
                request_id,
                spec,
                cancel_requested=cancelled,
                state_changed=lambda phase, attempt, current: self._publish_feedback(
                    goal_handle, phase, attempt, current, request_id,
                ),
            )

        try:
            if self._shutdown.is_set():
                raise RuntimeError("Application is shutting down")
            snapshot = await run_in_worker(self._admission, key, capture)
        except ReferenceCaptureCancelled as exception:
            state = self._terminal_state(
                goal,
                context,
                request_id,
                ProbeSetupState.STATE_CANCELLED,
                str(exception),
            )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
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
        finally:
            self._admission.release_pending(key)

        state = self.state_adapter.message(
            snapshot,
            ProbeSetupIntent.OPERATION_UNSPECIFIED,
            ProbeSetupState.STATE_SUCCEEDED,
            "Reference dataset capture completed",
            request_id=request_id,
        )
        deliver_update(self.node, self.state_publisher.publish, state)
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
        deliver_update(self.node, self.state_publisher.publish, state)
        feedback = CaptureProbeReferenceViews.Feedback()
        feedback.phase = self._phase_code(phase)
        feedback.attempt = int(attempt)
        feedback.state = state
        deliver_update(self.node, goal_handle.publish_feedback, feedback)

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
            and self.probe_setup_coordinator.is_current(context)
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
        deliver_update(self.node, self.state_publisher.publish, state)
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

    def request_shutdown(self):
        """Stop setup work before destroying its action transport."""
        self._shutdown.set()
        self.capture_coordinator.request_shutdown()

    def close(self):
        """Stop capture work and destroy its action transport."""
        resources = {
            "capture_coordinator": self.capture_coordinator.close,
            "action_server": self._action_server.destroy,
        }
        release_resources(self, resources)


__all__ = ["ProbeReferenceCaptureApi"]
