"""Execute one close-to-surface probe movement from live depth and kinematics."""

import math
import time
from dataclasses import dataclass, field
from threading import Event, RLock

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.inspection.execution.probe_surface_approach import (
    evaluate_probe_surface_approach,
    freeze_probe_surface_approach,
)
from fault_detector_spot.inspection.model.models import Vector3Data
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionFrame,
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    SurfaceVerificationState,
)


_TERMINAL_COMMAND_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
})


@dataclass
class _MotionWaiter:
    event: Event = field(default_factory=Event)
    status: object = None


class ProbeSurfaceVerificationRunner:
    """Run one bounded close-to-surface movement outside ROS transport."""

    def __init__(
        self,
        coordinator,
        state_source,
        settle_sec: float = 0.5,
        sample_timeout_sec: float = 3.0,
        poll_sec: float = 0.05,
        maximum_lateral_drift_m: float = 0.010,
        maximum_axis_error_rad: float = math.radians(5.0),
        minimum_step_progress_ratio: float = 0.25,
    ):
        if state_source is None:
            raise ValueError("Probe surface verification needs a state source")
        self.coordinator = coordinator
        self.state_source = state_source
        self.settle_sec = float(settle_sec)
        self.sample_timeout_sec = float(sample_timeout_sec)
        self.poll_sec = float(poll_sec)
        self.maximum_lateral_drift_m = float(maximum_lateral_drift_m)
        self.maximum_axis_error_rad = float(maximum_axis_error_rad)
        self.minimum_step_progress_ratio = float(minimum_step_progress_ratio)
        for value, label in (
            (self.settle_sec, "Surface settle time"),
            (self.sample_timeout_sec, "Surface sample timeout"),
            (self.poll_sec, "Surface workflow polling interval"),
            (self.maximum_lateral_drift_m, "Maximum lateral drift"),
            (self.maximum_axis_error_rad, "Maximum probe-axis error"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be positive")
        if (
            not math.isfinite(self.minimum_step_progress_ratio)
            or self.minimum_step_progress_ratio <= 0.0
            or self.minimum_step_progress_ratio > 1.0
        ):
            raise ValueError(
                "Minimum surface-step progress ratio must be in (0, 1]"
            )
        self._lock = RLock()
        self._waiters = {}
        coordinator.add_motion_status_listener(self._receive_motion_status)

    def run(
        self,
        context,
        request_id: str,
        cancel_requested,
        state_changed=None,
    ):
        """Run until target stand-off, failure, cancellation, or recovery."""
        if not callable(cancel_requested):
            raise TypeError("Cancellation predicate must be callable")
        if state_changed is not None and not callable(state_changed):
            raise TypeError("State listener must be callable")

        context_id = context.context_id
        client_id = context.client_id
        snapshot = self.coordinator.begin_surface_verification(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        receipt_not_before = time.monotonic()

        initial = self._wait_for_initial_evaluation(
            snapshot,
            request_id,
            receipt_not_before,
            cancel_requested,
        )
        if initial is None:
            return self._cancel(
                context_id,
                client_id,
                request_id,
                state_changed,
            )
        if isinstance(initial, Exception):
            return self._fail_sampling(
                context_id,
                client_id,
                request_id,
                initial,
                state_changed,
            )

        decision, snapshot, achieved_pose = initial
        self._emit(state_changed, snapshot)
        verification = snapshot.surface_verification
        if verification.state is SurfaceVerificationState.CONVERGED:
            return snapshot
        if not verification.active:
            return snapshot
        plan = freeze_probe_surface_approach(
            current_probe_pose_execution=achieved_pose,
            measured_initial_distance_m=decision.aggregate.distance_m,
            target_distance_m=verification.target_distance_m,
            maximum_travel_m=verification.maximum_cumulative_correction_m,
        )

        previous_pose = achieved_pose
        correction_m = decision.correction_m

        while True:
            if cancel_requested():
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )

            motion_result = self._execute_correction(
                context_id,
                client_id,
                request_id,
                correction_m,
                cancel_requested,
                state_changed,
            )
            if motion_result is None:
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )
            if isinstance(motion_result, Exception):
                return self._fail_correction(
                    context_id,
                    client_id,
                    request_id,
                    motion_result,
                    state_changed,
                )

            if not self._settle(cancel_requested):
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )

            context = self.coordinator.context(context_id, client_id)
            snapshot = self.coordinator.snapshot(context)
            current_pose = self.state_source.current_probe_pose_object(
                snapshot.selected_reference_tag_id,
                snapshot.selected_sensor_id,
            )
            evaluation = evaluate_probe_surface_approach(
                plan,
                current_probe_pose_execution=current_pose,
                maximum_step_m=verification.policy.maximum_step_m,
                tolerance_m=verification.policy.tolerance_m,
            )
            self._validate_progress(
                plan,
                previous_pose,
                current_pose,
                correction_m,
                evaluation,
            )
            previous_pose = current_pose

            decision, snapshot = self._evaluate_kinematic(
                context,
                request_id,
                evaluation.estimated_distance_m,
                current_pose,
            )
            self._emit(state_changed, snapshot)
            verification = snapshot.surface_verification
            if verification.state is SurfaceVerificationState.CONVERGED:
                return snapshot
            if not verification.active:
                return snapshot
            correction_m = decision.correction_m

    def _wait_for_initial_evaluation(
        self,
        snapshot,
        request_id,
        receipt_not_before,
        cancel_requested,
    ):
        deadline = time.monotonic() + self.sample_timeout_sec
        accumulated = {}
        last_error = None
        evaluated_sample_count = 0
        while True:
            if cancel_requested():
                return None
            try:
                fresh_samples = self.state_source.surface_distance_samples(
                    snapshot.selected_sensor_id,
                    receipt_not_before=receipt_not_before,
                    minimum_samples=1,
                )
                for sample in fresh_samples:
                    accumulated[sample.stamp_seconds] = sample
                if len(accumulated) > evaluated_sample_count:
                    achieved = self.state_source.current_probe_pose_object(
                        snapshot.selected_reference_tag_id,
                        snapshot.selected_sensor_id,
                    )
                    context = self.coordinator.context(
                        snapshot.context.context_id,
                        snapshot.context.client_id,
                    )
                    samples = tuple(
                        accumulated[stamp]
                        for stamp in sorted(accumulated)
                    )
                    evaluated_sample_count = len(samples)
                    decision, current = (
                        self.coordinator.evaluate_surface_verification(
                            context,
                            request_id,
                            samples,
                            achieved,
                        )
                    )
                    if not decision.resample_required:
                        return decision, current, achieved
            except Exception as exception:
                last_error = exception
            if time.monotonic() >= deadline:
                return last_error or RuntimeError(
                    "Timed out waiting for stable surface-distance samples"
                )
            time.sleep(self.poll_sec)

    def _execute_correction(
        self,
        context_id,
        client_id,
        request_id,
        correction_m,
        cancel_requested,
        state_changed,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.snapshot(context)
        operation = self._prepare_correction(
            snapshot,
            correction_m,
            request_id,
        )
        waiter = _MotionWaiter()
        with self._lock:
            self._waiters[operation.request_id] = waiter
        try:
            self.coordinator.submit_motion(operation)
        except Exception as exception:
            with self._lock:
                self._waiters.pop(operation.request_id, None)
            return exception

        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.mark_surface_correction_started(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        cancellation_sent = self._wait_for_motion(
            context_id,
            client_id,
            operation.request_id,
            waiter,
            cancel_requested,
        )
        with self._lock:
            self._waiters.pop(operation.request_id, None)

        status = waiter.status
        if cancellation_sent or (
            status is not None
            and status.state is CommandControllerState.CANCELLED
        ):
            return None
        if (
            status is None
            or status.state is not CommandControllerState.SUCCEEDED
        ):
            detail = (
                status.detail
                if status is not None
                else "Surface correction ended without a result"
            )
            return RuntimeError(detail)

        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.mark_surface_correction_succeeded(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        return snapshot

    def _evaluate_kinematic(
        self,
        context,
        request_id,
        estimated_distance_m,
        achieved_pose,
    ):
        with self.coordinator._context_lock(context):
            self.coordinator.setup_coordinator.require_current(context)
            draft = self.coordinator._selected_draft(context)
            decision = (
                self.coordinator.surface_controller.evaluate_estimated_distance(
                    draft,
                    request_id,
                    estimated_distance_m,
                    achieved_pose,
                )
            )
            return decision, self.coordinator.snapshot(context)

    def _validate_progress(
        self,
        plan,
        previous_pose,
        current_pose,
        requested_step_m,
        evaluation,
    ):
        if evaluation.lateral_offset_m > self.maximum_lateral_drift_m:
            raise RuntimeError(
                "Surface approach lateral drift exceeded the safety limit: "
                f"{evaluation.lateral_offset_m:.4f} m"
            )
        if evaluation.axis_error_rad > self.maximum_axis_error_rad:
            raise RuntimeError(
                "Probe axis rotated away from the frozen surface normal by "
                f"{math.degrees(evaluation.axis_error_rad):.2f} deg"
            )
        inward = plan.inward_direction()
        delta = Vector3Data(
            x=current_pose.position.x - previous_pose.position.x,
            y=current_pose.position.y - previous_pose.position.y,
            z=current_pose.position.z - previous_pose.position.z,
        )
        achieved_step_m = (
            delta.x * inward.x
            + delta.y * inward.y
            + delta.z * inward.z
        )
        minimum_progress = (
            requested_step_m * self.minimum_step_progress_ratio
        )
        if achieved_step_m + 1e-9 < minimum_progress:
            raise RuntimeError(
                "Surface approach did not achieve enough inward progress: "
                f"requested {requested_step_m:.4f} m, "
                f"achieved {achieved_step_m:.4f} m. "
                "Possible obstruction or contact."
            )

    def _prepare_correction(
        self,
        snapshot,
        correction_m,
        request_id,
    ):
        motion = ProbeMotionRequest(
            kind=ProbeMotionKind.ADJUST_PROBE_DISTANCE,
            frame=ProbeMotionFrame.SENSOR,
            translation=Vector3Data(
                x=float(correction_m),
                y=0.0,
                z=0.0,
            ),
            position_tolerance_m=0.005,
            orientation_tolerance_rad=0.05,
        )
        return self.coordinator.prepare_motion(
            snapshot.context,
            motion,
            surface_verification_request_id=request_id,
        )

    def _wait_for_motion(
        self,
        context_id,
        client_id,
        request_id,
        waiter,
        cancel_requested,
    ):
        cancellation_sent = False
        while not waiter.event.wait(self.poll_sec):
            if cancel_requested() and not cancellation_sent:
                cancellation_sent = True
                try:
                    context = self.coordinator.context(
                        context_id,
                        client_id,
                    )
                    self.coordinator.cancel_motion(context, request_id)
                except LookupError:
                    pass
        return cancellation_sent

    def _settle(self, cancel_requested):
        deadline = time.monotonic() + self.settle_sec
        while True:
            if cancel_requested():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(self.poll_sec, remaining))

    def _cancel(
        self,
        context_id,
        client_id,
        request_id,
        state_changed,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.cancel_surface_verification(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        return snapshot

    def _fail_sampling(
        self,
        context_id,
        client_id,
        request_id,
        exception,
        state_changed,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.fail_surface_sampling(
            context,
            request_id,
            str(exception),
        )
        self._emit(state_changed, snapshot)
        return snapshot

    def _fail_correction(
        self,
        context_id,
        client_id,
        request_id,
        exception,
        state_changed,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.fail_surface_correction(
            context,
            request_id,
            str(exception),
        )
        self._emit(state_changed, snapshot)
        return snapshot

    def _receive_motion_status(self, status):
        with self._lock:
            waiter = self._waiters.get(status.request_id)
            if waiter is None:
                return
            waiter.status = status
            if status.state in _TERMINAL_COMMAND_STATES:
                waiter.event.set()

    @staticmethod
    def _emit(listener, snapshot):
        if listener is not None:
            listener(snapshot)

    def close(self):
        self.coordinator.remove_motion_status_listener(
            self._receive_motion_status
        )
        with self._lock:
            waiters = tuple(self._waiters.values())
            self._waiters.clear()
        for waiter in waiters:
            waiter.event.set()


__all__ = ["ProbeSurfaceVerificationRunner"]
