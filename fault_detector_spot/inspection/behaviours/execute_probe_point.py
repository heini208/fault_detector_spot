"""Execute one contextual measurement through existing motion owners."""

import time

from py_trees.common import Status
from fault_detector_msgs.srv import ProbePointExecutionStep

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.semantic_command_adapter import semantic_command_from_message
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import ArmMovementBehaviour
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome
from fault_detector_spot.manipulation.move_close_to_surface_execution import (
    MoveCloseToSurfaceConfig, MoveCloseToSurfaceExecution, MoveCloseToSurfaceOutcome,
)


class ExecuteProbePoint(ArmMovementBehaviour):
    """Own stage ordering and bounded retries; executors own physical safety."""

    TAG_REACQUIRE_TIMEOUT_SEC = 5.0

    STAGES = ("Safe approach", "Aligned preapproach", "Measurement pose")
    # These outcomes prohibit autonomous recovery, regardless of retry budget.
    UNSAFE_TO_RECOVER = frozenset({
        ArmMovementOutcome.STOP_UNCONFIRMED, ArmMovementOutcome.RETREAT_FAILED,
        ArmMovementOutcome.RECOVERY_FAILED, ArmMovementOutcome.UNSTABLE_ARM,
        ArmMovementOutcome.BUSY, ArmMovementOutcome.TRAJECTORY_CANCELLED,
    })

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ensure_executor()
        self._client = self.node.create_client(
            ProbePointExecutionStep, "fault_detector/_internal/probe_point_execution",
        )
        self._surface = MoveCloseToSurfaceExecution(
            self.executor, self.robot_command_resources.get_probe_surface_source(self.node),
            MoveCloseToSurfaceConfig.from_node(self.node), logger=self.node.get_logger(),
        )
        self._builder = CommandSubscriber()
        self._builder.node = self.node

    def initialise(self):
        super().initialise()
        self._tag_wait_started = None
        self._phase = "plan"
        self._future = None
        self._rpc_started = time.monotonic()
        self._attempt = 0
        self._stage = 0
        self._index = 0
        self._steps = None
        self._sensor_id = ""
        self._history = []
        self._failure = ""
        self._terminal_failure = ""
        self._resume_phase = "motion"
        self._retry_pending = False
        self._stop_started = False
        self._stop_deadline = None
        self._record_started = False
        self._abort_started = False
        self._abort_deadline = None

    def _rpc(self, operation):
        if self._future is None:
            if not self._client.service_is_ready():
                if time.monotonic() - self._rpc_started > 10:
                    raise RuntimeError("Probe execution service unavailable")
                return None
            request = ProbePointExecutionStep.Request()
            request.request_id = self._current_request_id()
            request.operation = operation
            self._future = self._client.call_async(request)
            self._rpc_started = time.monotonic()
        if not self._future.done():
            if time.monotonic() - self._rpc_started > 10:
                raise RuntimeError("Probe execution service timed out")
            return None
        response = self._future.result()
        self._future = None
        self._rpc_started = time.monotonic()
        return response

    def update(self):
        try:
            return self._advance()
        except Exception as exception:
            if self._phase == "motion" and self._history:
                return self._begin_failure(str(exception), "motion")
            # Unknown recording/transport state or a failed recovery must not
            # lead to a second recording or a shortcut through the return path.
            return self._fail(str(exception))

    def _advance(self):
        if self._phase == "plan":
            response = self._rpc(ProbePointExecutionStep.Request.PLAN)
            if response is None:
                return Status.RUNNING
            if response.recording_state == "waiting_tag":
                return self._wait_for_tag(response.detail, "plan")
            self._tag_wait_started = None
            if not response.success:
                return self._fail(response.detail)
            if len(response.plan) != 3:
                raise ValueError("Probe execution requires three forward motion stages")
            plan = [semantic_command_from_message(p) for p in response.plan]
            self._sensor_id = plan[0].motion_sensor_id
            if not self._sensor_id or any(p.motion_sensor_id != self._sensor_id for p in plan):
                raise ValueError("Probe execution plan has inconsistent attachment geometry")
            self._steps = [self._builder.fire_command_sequence(p) for p in plan]
            # Capture before any movement, including automatic arm deployment.
            self._history = [self.executor.capture_probe_checkpoint(self._sensor_id)]
            self._phase = "motion"

        if self._phase == "record":
            return self._record()
        if self._phase == "abort_record":
            return self._abort_record()
        if self._phase == "stop":
            return self._confirm_stop()
        if self._phase == "checkpoint":
            return self._recover_checkpoint()
        if self._phase == "return":
            return self._backtrack()
        return self._move_forward()

    def _move_forward(self):
        step = self._steps[self._stage][self._index]
        if not self._started and hasattr(step, "tag_id"):
            if self.executor.tag_state_source.usable_tag(step.tag_id) is None:
                return self._wait_for_tag(f"Tag {step.tag_id} is not currently usable", "motion")
            self._tag_wait_started = None
        surface = step.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
        if surface:
            outcome = self._surface.poll() if self._started else self._surface.start(step)
            self._started = outcome is MoveCloseToSurfaceOutcome.RUNNING
            success = outcome is MoveCloseToSurfaceOutcome.SUCCESS
            detail = self._surface.feedback_message
            failure_outcome = self._surface.failure_outcome
        else:
            update = (self.executor.poll() if self._started else self.executor.tag_probe(
                step, speed=self.executor.safe_approach_speed if self._stage == 0 else None,
            ))
            self._started = update.outcome is ArmMovementOutcome.RUNNING
            success = update.outcome is ArmMovementOutcome.SUCCESS
            detail = update.detail
            failure_outcome = update.outcome
        self.feedback_message = f"{self.STAGES[self._stage]} (retries used: {self._attempt}): {detail}"
        if self._started:
            return Status.RUNNING
        if not success:
            if failure_outcome in self.UNSAFE_TO_RECOVER:
                return self._fail(self.feedback_message)
            return self._begin_failure(self.feedback_message, "motion")
        self._history.append(self.executor.capture_probe_checkpoint(self._sensor_id))
        self._index += 1
        if self._index >= len(self._steps[self._stage]):
            self._stage += 1
            self._index = 0
            if self._stage == len(self._steps):
                self._phase = "record"
        return Status.RUNNING

    def _record(self):
        response = self._rpc(ProbePointExecutionStep.Request.POLL if self._record_started
                             else ProbePointExecutionStep.Request.RECORD)
        if response is None:
            return Status.RUNNING
        if response.recording_state == "waiting_tag":
            return self._wait_for_tag(response.detail, "record")
        self._tag_wait_started = None
        self._record_started = True
        self.feedback_message = response.detail
        if not response.success or response.recording_state == "failed":
            self._failure = "Probe recording failed: " + response.detail
            self._phase = "abort_record"
            self._abort_started = False
            self._abort_deadline = time.monotonic() + 10.0
            return Status.RUNNING
        if response.recording_state == "complete":
            if not response.recording_stopped:
                return self._fail("Sensor stop unconfirmed; backtracking prohibited")
            self._phase = "return"
        return Status.RUNNING

    def _abort_record(self):
        response = self._rpc(ProbePointExecutionStep.Request.POLL if self._abort_started
                             else ProbePointExecutionStep.Request.ABORT_RECORDING)
        if response is None:
            return Status.RUNNING
        self._abort_started = True
        if not response.success:
            return self._fail("Cannot confirm failed recording stopped: " + response.detail)
        if response.recording_stopped:
            return self._begin_failure(self._failure, "record")
        if response.recording_state == "failed" or time.monotonic() >= self._abort_deadline:
            return self._fail("Sensor stop unconfirmed; retry and backtracking prohibited")
        return Status.RUNNING

    def _wait_for_tag(self, detail, resume_phase):
        now = time.monotonic()
        if self._tag_wait_started is None:
            self._tag_wait_started = now
        self.feedback_message = "Waiting for tag reacquisition: " + detail
        if now - self._tag_wait_started < self.TAG_REACQUIRE_TIMEOUT_SEC:
            return Status.RUNNING
        self._tag_wait_started = None
        detail = "Tag reacquisition timed out: " + detail
        if resume_phase == "plan":
            # No motion has started and there is no checkpoint to recover yet.
            if self._attempt < self._last_command().retries:
                self._attempt += 1
                return Status.RUNNING
            return self._fail(detail)
        return self._begin_failure(detail, resume_phase)

    def _begin_failure(self, detail, resume_phase):
        self._failure = detail
        self._resume_phase = resume_phase
        self._retry_pending = self._attempt < self._last_command().retries
        if self._retry_pending:
            self._attempt += 1
        self._started = False
        self._stop_started = False
        self._stop_deadline = time.monotonic() + 10.0
        self._phase = "stop"
        if self._surface.active:
            self._surface.cancel()
        if self.executor.active:
            self.executor.cancel()
        return Status.RUNNING

    def _confirm_stop(self):
        if not self._stop_started and self.executor.active:
            self.executor.poll()
            if time.monotonic() >= self._stop_deadline:
                return self._fail("Stop unconfirmed; recovery prohibited")
            return Status.RUNNING
        update = self.executor.poll() if self._stop_started else self.executor.confirm_stop()
        self._stop_started = True
        if update.outcome is ArmMovementOutcome.RUNNING:
            return Status.RUNNING
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            return self._fail("Stop unconfirmed; recovery prohibited: " + update.detail)
        self._phase = "checkpoint"
        return Status.RUNNING

    def _recover_checkpoint(self):
        update = (self.executor.poll() if self._started
                  else self.executor.restore_probe_checkpoint(self._history[-1]))
        self._started = update.outcome is ArmMovementOutcome.RUNNING
        self.feedback_message = "Returning to last successful goal: " + update.detail
        if self._started:
            return Status.RUNNING
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            if (update.outcome is ArmMovementOutcome.CHECKPOINT_TOLERANCE_FAILED
                    and self._attempt < self._last_command().retries):
                # Retry this same recovery target only after another confirmed stop.
                # Keep the original failed step and its reserved retry unchanged.
                self._attempt += 1
                self._stop_started = False
                self._stop_deadline = time.monotonic() + 10.0
                self._phase = "stop"
                self.feedback_message = (
                    f"Retrying checkpoint recovery (retries used: {self._attempt}): "
                    + update.detail
                )
                return Status.RUNNING
            detail = "Checkpoint recovery failed; further motion prohibited: " + update.detail
            if update.outcome is ArmMovementOutcome.CHECKPOINT_TOLERANCE_FAILED:
                detail += f"; retry budget exhausted ({self._attempt}/{self._last_command().retries})"
            return self._fail(detail)
        if self._retry_pending:
            self._phase = self._resume_phase
            self._record_started = False
        elif self._resume_phase == "return":
            return self._fail(self._failure + "; return blocked; stopped at last successful goal")
        else:
            self._terminal_failure = self._failure
            self._phase = "return"
        return Status.RUNNING

    def _backtrack(self):
        # The pre-command pose is only a recovery target before safe approach
        # succeeds. Normal backtracking ends at the first reached checkpoint.
        if len(self._history) <= 2:
            destination = "safe approach" if len(self._history) == 2 else "initial pose"
            if self._terminal_failure:
                return self._fail(self._terminal_failure + f"; retries exhausted; returned to {destination}")
            self.feedback_message = "Probe measurement recorded; returned to safe approach"
            return Status.SUCCESS
        # Pop only after the preceding checkpoint was successfully reached.
        # A failed return therefore recovers to the last reached checkpoint.
        update = (self.executor.poll() if self._started
                  else self.executor.restore_probe_checkpoint(self._history[-2]))
        self._started = update.outcome is ArmMovementOutcome.RUNNING
        self.feedback_message = "Backtracking to safe approach: " + update.detail
        if self._started:
            return Status.RUNNING
        if update.outcome is not ArmMovementOutcome.SUCCESS:
            if update.outcome in self.UNSAFE_TO_RECOVER:
                return self._fail(self.feedback_message)
            return self._begin_failure(self.feedback_message, "return")
        self._history.pop()
        return Status.RUNNING

    def _fail(self, detail):
        if self._surface.active:
            self._surface.cancel()
        if self.executor.active:
            self.executor.cancel()
        return super()._fail(detail)

    def terminate(self, new_status):
        if new_status is Status.INVALID:
            if self._surface.active:
                self._surface.cancel()
            if self.executor.active:
                self.executor.cancel()
        super().terminate(new_status)

    def shutdown(self):
        if hasattr(self, "_surface") and self._surface.active:
            self._surface.cancel()
        super().shutdown()
        if hasattr(self, "_client"):
            self.node.destroy_client(self._client)
