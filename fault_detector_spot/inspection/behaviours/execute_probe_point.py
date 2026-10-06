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

    STAGES = ("Safe approach", "Aligned preapproach", "Measurement pose",
              "Return to aligned preapproach", "Return to safe approach")
    RETRYABLE = frozenset({ArmMovementOutcome.PLANNING_FAILED,
                          ArmMovementOutcome.GOAL_REJECTED,
                          ArmMovementOutcome.ACTION_SERVER_UNAVAILABLE})

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
        self._phase = "plan"
        self._future = None
        self._rpc_started = time.monotonic()
        self._attempt = 0
        self._stage = 0
        self._index = 0
        self._steps = None
        self._recovery = []
        self._failure = ""
        self._retry_pending = False
        self._stop_started = False
        self._stop_deadline = None
        self._record_started = False

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
        if not response.success:
            raise RuntimeError(response.detail)
        return response

    def update(self):
        try:
            return self._advance()
        except Exception as exception:
            return self._fail(str(exception))

    def _advance(self):
        if self._phase == "plan":
            response = self._rpc(ProbePointExecutionStep.Request.PLAN)
            if response is None:
                return Status.RUNNING
            if len(response.plan) != 5:
                raise ValueError("Probe execution requires five motion stages")
            self._steps = [self._builder.fire_command_sequence(semantic_command_from_message(p))
                           for p in response.plan]
            self._phase = "motion"

        if self._phase == "record":
            response = self._rpc(ProbePointExecutionStep.Request.POLL if self._record_started
                                 else ProbePointExecutionStep.Request.RECORD)
            if response is None:
                return Status.RUNNING
            self._record_started = True
            self.feedback_message = response.detail
            if response.recording_state == "failed":
                return self._fail("Probe recording failed: " + response.detail)
            if response.recording_state != "complete":
                return Status.RUNNING
            self._stage, self._index = 3, 0
            self._phase = "motion"

        if self._phase == "stop":
            # A cancellation remains active until the executor confirms its stop.
            if not self._stop_started and self.executor.active:
                self.executor.poll()
                if time.monotonic() >= self._stop_deadline:
                    return self._fail("Stop unconfirmed; probe retry prohibited")
                return Status.RUNNING
            update = self.executor.poll() if self._stop_started else self.executor.confirm_stop()
            self._stop_started = True
            if update.outcome is ArmMovementOutcome.RUNNING:
                return Status.RUNNING
            if update.outcome is not ArmMovementOutcome.SUCCESS:
                return self._fail("Stop unconfirmed; probe retry prohibited: " + update.detail)
            self._phase = "recovery"
            self._index = 0

        if self._phase == "recovery":
            if self._index >= len(self._recovery):
                if not self._retry_pending:
                    return self._fail(self._failure + "; recovered to safe approach; retries exhausted")
                self._attempt += 1
                self._stage, self._index = 0, 0
                self._phase = "motion"
            else:
                update = (self.executor.poll() if self._started
                          else self.executor.tag_probe(self._recovery[self._index],
                                                       speed=self.executor.safe_approach_speed))
                self._started = update.outcome is ArmMovementOutcome.RUNNING
                self.feedback_message = "Recovering before retry: " + update.detail
                if self._started:
                    return Status.RUNNING
                if update.outcome is not ArmMovementOutcome.SUCCESS:
                    return self._fail("Probe recovery failed; retry prohibited: " + update.detail)
                self._index += 1
                return Status.RUNNING

        if self._stage == 5:
            self.feedback_message = "Probe measurement recorded and arm returned to safe approach"
            return Status.SUCCESS
        step = self._steps[self._stage][self._index]
        surface = step.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
        if surface:
            outcome = self._surface.poll() if self._started else self._surface.start(step)
            self._started = outcome is MoveCloseToSurfaceOutcome.RUNNING
            success = outcome is MoveCloseToSurfaceOutcome.SUCCESS
            detail = self._surface.feedback_message
            retryable = self._surface.retry_eligible
        else:
            update = (self.executor.poll() if self._started else self.executor.tag_probe(
                step, speed=self.executor.safe_approach_speed if self._stage in (0, 4) else None,
            ))
            self._started = update.outcome is ArmMovementOutcome.RUNNING
            success = update.outcome is ArmMovementOutcome.SUCCESS
            detail = update.detail
            retryable = update.outcome in self.RETRYABLE
        self.feedback_message = f"Attempt {self._attempt + 1}: {self.STAGES[self._stage]}: {detail}"
        if self._started:
            return Status.RUNNING
        if not success:
            retries = self._last_command().retries
            if not retryable or self._stage >= 3:
                return self._fail(self.feedback_message)
            self._retry_pending = self._attempt < retries
            self._failure = self.feedback_message
            # Retrace only reached waypoints, never unvisited forward waypoints.
            self._recovery = list(reversed(self._steps[self._stage][:self._index])) if not surface else []
            if self._stage == 2:
                self._recovery += [self._steps[1][-1]]
                self._recovery += self._steps[4]
            elif self._stage <= 1:
                self._recovery += self._steps[0]
            self._phase = "stop"
            self._stop_started = False
            self._stop_deadline = time.monotonic() + 10
            if self.executor.active:
                self.executor.cancel()
            return Status.RUNNING
        self._index += 1
        if self._index >= len(self._steps[self._stage]):
            if self._stage == 2:
                self._phase = "record"
            else:
                self._stage += 1
            self._index = 0
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
