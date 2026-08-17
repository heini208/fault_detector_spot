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
from fault_detector_spot.inspection.sensing.end_effector_force import (
    estimate_force_baseline,
    project_probe_force_delta,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionFrame,
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    SurfaceVerificationState,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    rotate_vector,
)


_TERMINAL_COMMAND_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
})


class _ProbableContact(RuntimeError):
    pass


class _ForceGuardFailure(RuntimeError):
    pass


@dataclass
class _MotionWaiter:
    event: Event = field(default_factory=Event)
    status: object = None


@dataclass
class _ForceGuardState:
    last_receipt_time: float
    consecutive_contact_samples: int = 0


@dataclass(frozen=True)
class _AttemptOutcome:
    kind: str
    snapshot: object = None
    exception: Exception = None


class ProbeSurfaceVerificationRunner:
    """Run one bounded close-to-surface movement outside ROS transport."""

    def __init__(
        self,
        coordinator,
        state_source,
        sensor_attachment_controller=None,
        settle_sec: float = 0.5,
        sample_timeout_sec: float = 3.0,
        poll_sec: float = 0.05,
        maximum_lateral_drift_m: float = 0.010,
        maximum_axis_error_rad: float = math.radians(5.0),
        minimum_step_progress_ratio: float = 0.25,
        force_contact_threshold_n: float = 5.0,
        force_contact_consecutive_samples: int = 2,
        force_baseline_timeout_sec: float = 2.0,
        force_stale_timeout_sec: float = 0.5,
        maximum_contact_retries: int = 3,
        recovery_step_m: float = 0.04,
        maximum_recovery_steps: int = 20,
    ):
        if state_source is None:
            raise ValueError("Probe surface verification needs a state source")
        self.coordinator = coordinator
        self.state_source = state_source
        self.sensor_attachment_controller = sensor_attachment_controller
        self.settle_sec = float(settle_sec)
        self.sample_timeout_sec = float(sample_timeout_sec)
        self.poll_sec = float(poll_sec)
        self.maximum_lateral_drift_m = float(maximum_lateral_drift_m)
        self.maximum_axis_error_rad = float(maximum_axis_error_rad)
        self.minimum_step_progress_ratio = float(minimum_step_progress_ratio)
        self.force_contact_threshold_n = float(force_contact_threshold_n)
        self.force_baseline_timeout_sec = float(force_baseline_timeout_sec)
        self.force_stale_timeout_sec = float(force_stale_timeout_sec)
        self.recovery_step_m = float(recovery_step_m)
        for value, label in (
            (self.settle_sec, "Surface settle time"),
            (self.sample_timeout_sec, "Surface sample timeout"),
            (self.poll_sec, "Surface workflow polling interval"),
            (self.maximum_lateral_drift_m, "Maximum lateral drift"),
            (self.maximum_axis_error_rad, "Maximum probe-axis error"),
            (self.force_contact_threshold_n, "Force contact threshold"),
            (self.force_baseline_timeout_sec, "Force baseline timeout"),
            (self.force_stale_timeout_sec, "Force stale timeout"),
            (self.recovery_step_m, "Surface recovery step"),
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
        self.force_contact_consecutive_samples = self._positive_integer(
            force_contact_consecutive_samples,
            "Force contact consecutive sample count",
        )
        self.maximum_contact_retries = self._non_negative_integer(
            maximum_contact_retries,
            "Maximum surface contact retries",
        )
        self.maximum_recovery_steps = self._positive_integer(
            maximum_recovery_steps,
            "Maximum surface recovery steps",
        )
        if self.recovery_step_m > 0.05 + 1e-12:
            raise ValueError("Surface recovery step must not exceed 0.05 m")
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
        if self.sensor_attachment_controller is None:
            raise RuntimeError(
                "Active sensor attachment state is unavailable"
            )
        with (
            self.sensor_attachment_controller.reserve_motion_attachment()
            as attachment
        ):
            return self._run_reserved(
                context,
                request_id,
                cancel_requested,
                state_changed,
                attachment,
            )

    def _run_reserved(
        self,
        context,
        request_id,
        cancel_requested,
        state_changed,
        attachment,
    ):
        if not callable(cancel_requested):
            raise TypeError("Cancellation predicate must be callable")
        if state_changed is not None and not callable(state_changed):
            raise TypeError("State listener must be callable")

        sensor_id = attachment.motion_sensor_id
        probe_axis_hand = rotate_vector(
            attachment.hand_to_probe().orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        context_id = context.context_id
        client_id = context.client_id
        snapshot = self.coordinator.begin_surface_verification(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        recovery_pose = None
        retries_used = 0

        while True:
            receipt_not_before = time.monotonic()
            initial = self._wait_for_initial_evaluation(
                snapshot,
                request_id,
                receipt_not_before,
                cancel_requested,
                sensor_id,
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
            if recovery_pose is None:
                recovery_pose = achieved_pose
            verification = snapshot.surface_verification
            if verification.state is SurfaceVerificationState.CONVERGED:
                return snapshot
            if not verification.active:
                return snapshot

            try:
                plan = freeze_probe_surface_approach(
                    current_probe_pose_execution=achieved_pose,
                    measured_initial_distance_m=decision.aggregate.distance_m,
                    target_distance_m=verification.target_distance_m,
                    maximum_travel_m=(
                        verification.maximum_cumulative_correction_m
                    ),
                )
            except Exception as exception:
                return self._abort(
                    context_id,
                    client_id,
                    request_id,
                    exception,
                    state_changed,
                )

            baseline_result = self._wait_for_force_baseline(
                cancel_requested,
            )
            if baseline_result is None:
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )
            if isinstance(baseline_result, Exception):
                return self._abort(
                    context_id,
                    client_id,
                    request_id,
                    baseline_result,
                    state_changed,
                )
            baseline, last_force_receipt_time = baseline_result
            force_state = _ForceGuardState(last_force_receipt_time)

            outcome = self._run_approach_attempt(
                context_id,
                client_id,
                request_id,
                sensor_id,
                plan,
                decision.correction_m,
                baseline,
                probe_axis_hand,
                force_state,
                cancel_requested,
                state_changed,
            )
            if outcome.kind in {"complete", "terminal"}:
                return outcome.snapshot
            if outcome.kind == "cancel":
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )

            recovered = self._recover_to_preapproach(
                context_id,
                client_id,
                request_id,
                sensor_id,
                recovery_pose,
                outcome.exception,
                cancel_requested,
                state_changed,
            )
            if recovered is None:
                return self._cancel(
                    context_id,
                    client_id,
                    request_id,
                    state_changed,
                )
            if isinstance(recovered, Exception):
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.snapshot(context)
                self._emit(state_changed, snapshot)
                return snapshot
            snapshot = recovered

            if outcome.kind == "force_failure":
                return self._fail_sampling(
                    context_id,
                    client_id,
                    request_id,
                    outcome.exception,
                    state_changed,
                )
            if retries_used >= self.maximum_contact_retries:
                return self._fail_sampling(
                    context_id,
                    client_id,
                    request_id,
                    RuntimeError(
                        "Possible surface contact persisted after "
                        f"{self.maximum_contact_retries} retries"
                    ),
                    state_changed,
                )
            retries_used += 1

    def _run_approach_attempt(
        self,
        context_id,
        client_id,
        request_id,
        sensor_id,
        plan,
        correction_m,
        baseline,
        probe_axis_hand,
        force_state,
        cancel_requested,
        state_changed,
    ):
        previous_pose = self._current_probe_pose(
            context_id,
            client_id,
            sensor_id,
        )
        while True:
            if cancel_requested():
                return _AttemptOutcome("cancel")

            motion_result = self._execute_correction(
                context_id,
                client_id,
                request_id,
                correction_m,
                baseline,
                probe_axis_hand,
                force_state,
                cancel_requested,
                state_changed,
            )
            if motion_result is None:
                return _AttemptOutcome("cancel")
            if isinstance(motion_result, _ProbableContact):
                return _AttemptOutcome(
                    "contact",
                    exception=motion_result,
                )
            if isinstance(motion_result, _ForceGuardFailure):
                return _AttemptOutcome(
                    "force_failure",
                    exception=motion_result,
                )
            if isinstance(motion_result, Exception):
                snapshot = self._fail_correction(
                    context_id,
                    client_id,
                    request_id,
                    motion_result,
                    state_changed,
                )
                return _AttemptOutcome("terminal", snapshot=snapshot)

            settle_result = self._settle_with_force_guard(
                cancel_requested,
                baseline,
                probe_axis_hand,
                force_state,
            )
            if settle_result is None:
                return _AttemptOutcome("cancel")
            if isinstance(settle_result, _ProbableContact):
                return _AttemptOutcome(
                    "contact",
                    exception=settle_result,
                )
            if isinstance(settle_result, _ForceGuardFailure):
                return _AttemptOutcome(
                    "force_failure",
                    exception=settle_result,
                )

            context = self.coordinator.context(context_id, client_id)
            snapshot = self.coordinator.snapshot(context)
            current_pose = self.state_source.current_probe_pose_object(
                snapshot.selected_reference_tag_id,
                sensor_id,
            )
            try:
                evaluation = evaluate_probe_surface_approach(
                    plan,
                    current_probe_pose_execution=current_pose,
                    maximum_step_m=(
                        snapshot.surface_verification.policy.maximum_step_m
                    ),
                    tolerance_m=(
                        snapshot.surface_verification.policy.tolerance_m
                    ),
                )
                self._validate_progress(
                    plan,
                    previous_pose,
                    current_pose,
                    correction_m,
                    evaluation,
                )
            except _ProbableContact as exception:
                return _AttemptOutcome(
                    "contact",
                    exception=exception,
                )
            except Exception as exception:
                snapshot = self._abort(
                    context_id,
                    client_id,
                    request_id,
                    exception,
                    state_changed,
                )
                return _AttemptOutcome("terminal", snapshot=snapshot)
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
                return _AttemptOutcome("complete", snapshot=snapshot)
            if not verification.active:
                return _AttemptOutcome("terminal", snapshot=snapshot)
            correction_m = decision.correction_m

    def _wait_for_initial_evaluation(
        self,
        snapshot,
        request_id,
        receipt_not_before,
        cancel_requested,
        sensor_id,
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
                    sensor_id,
                    receipt_not_before=receipt_not_before,
                    minimum_samples=1,
                )
                for sample in fresh_samples:
                    accumulated[sample.stamp_seconds] = sample
                if len(accumulated) > evaluated_sample_count:
                    achieved = self.state_source.current_probe_pose_object(
                        snapshot.selected_reference_tag_id,
                        sensor_id,
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

    def _wait_for_force_baseline(self, cancel_requested):
        receipt_not_before = time.monotonic()
        deadline = receipt_not_before + self.force_baseline_timeout_sec
        last_error = None
        while True:
            if cancel_requested():
                return None
            try:
                samples = self.state_source.end_effector_force_samples(
                    receipt_not_before=receipt_not_before,
                    maximum_age_sec=self.force_baseline_timeout_sec,
                )
                baseline = estimate_force_baseline(samples)
                newest_receipt = max(
                    sample.receipt_time
                    for sample in samples
                )
                return baseline, newest_receipt
            except Exception as exception:
                last_error = exception
            if time.monotonic() >= deadline:
                detail = (
                    str(last_error)
                    if last_error is not None
                    else "No fresh end-effector force samples are available"
                )
                return _ForceGuardFailure(
                    "Unable to establish a stationary end-effector force "
                    f"baseline: {detail}"
                )
            time.sleep(self.poll_sec)

    def _execute_correction(
        self,
        context_id,
        client_id,
        request_id,
        correction_m,
        baseline,
        probe_axis_hand,
        force_state,
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
        user_cancelled, force_exception = self._wait_for_motion(
            context_id,
            client_id,
            operation.request_id,
            waiter,
            cancel_requested,
            baseline,
            probe_axis_hand,
            force_state,
        )
        with self._lock:
            self._waiters.pop(operation.request_id, None)

        if user_cancelled:
            return None
        if force_exception is not None:
            return force_exception
        status = waiter.status
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
        return self.coordinator.evaluate_surface_estimate(
            context,
            request_id,
            estimated_distance_m,
            achieved_pose,
        )

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
            raise _ProbableContact(
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
        baseline,
        probe_axis_hand,
        force_state,
    ):
        user_cancelled = False
        force_exception = None
        cancellation_sent = False
        while not waiter.event.wait(self.poll_sec):
            if cancel_requested() and not cancellation_sent:
                user_cancelled = True
                cancellation_sent = True
                self._cancel_motion(
                    context_id,
                    client_id,
                    request_id,
                )
                continue
            if cancellation_sent:
                continue
            force_exception = self._check_force_guard(
                baseline,
                probe_axis_hand,
                force_state,
            )
            if force_exception is not None:
                cancellation_sent = True
                self._cancel_motion(
                    context_id,
                    client_id,
                    request_id,
                )
        return user_cancelled, force_exception

    def _check_force_guard(
        self,
        baseline,
        probe_axis_hand,
        force_state,
    ):
        now = time.monotonic()
        try:
            sample = self.state_source.latest_end_effector_force()
        except Exception as exception:
            if (
                now - force_state.last_receipt_time
                > self.force_stale_timeout_sec
            ):
                return _ForceGuardFailure(
                    "End-effector force became stale during the surface "
                    f"approach: {exception}"
                )
            return None

        if sample.receipt_time <= force_state.last_receipt_time + 1e-12:
            if (
                now - force_state.last_receipt_time
                > self.force_stale_timeout_sec
            ):
                return _ForceGuardFailure(
                    "End-effector force stopped updating during the surface "
                    "approach"
                )
            return None

        force_state.last_receipt_time = sample.receipt_time
        try:
            delta = project_probe_force_delta(
                sample,
                baseline,
                probe_axis_hand,
            )
        except Exception as exception:
            return _ForceGuardFailure(
                f"Unable to evaluate end-effector force: {exception}"
            )

        if delta.total_force_n >= self.force_contact_threshold_n:
            force_state.consecutive_contact_samples += 1
        else:
            force_state.consecutive_contact_samples = 0
        if (
            force_state.consecutive_contact_samples
            < self.force_contact_consecutive_samples
        ):
            return None
        return _ProbableContact(
            "End-effector force indicates possible contact: "
            f"total delta {delta.total_force_n:.2f} N, "
            f"axial {delta.axial_force_n:+.2f} N, "
            f"lateral {delta.lateral_force_n:.2f} N"
        )

    def _settle_with_force_guard(
        self,
        cancel_requested,
        baseline,
        probe_axis_hand,
        force_state,
    ):
        deadline = time.monotonic() + self.settle_sec
        while True:
            if cancel_requested():
                return None
            force_exception = self._check_force_guard(
                baseline,
                probe_axis_hand,
                force_state,
            )
            if force_exception is not None:
                return force_exception
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(self.poll_sec, remaining))

    def _recover_to_preapproach(
        self,
        context_id,
        client_id,
        request_id,
        sensor_id,
        recovery_pose,
        exception,
        cancel_requested,
        state_changed,
    ):
        snapshot = self._abort(
            context_id,
            client_id,
            request_id,
            exception,
            state_changed,
        )
        if (
            snapshot.surface_verification.state
            is not SurfaceVerificationState.RECOVERY_REQUIRED
        ):
            return RuntimeError(
                "Surface contact did not enter recovery state"
            )

        retraction_result = self._retract_to_pose(
            context_id,
            client_id,
            request_id,
            sensor_id,
            recovery_pose,
            cancel_requested,
        )
        if retraction_result is None:
            return None
        if isinstance(retraction_result, Exception):
            return retraction_result

        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.resume_surface_sampling(
            context,
            request_id,
        )
        self._emit(state_changed, snapshot)
        return snapshot

    def _retract_to_pose(
        self,
        context_id,
        client_id,
        request_id,
        sensor_id,
        recovery_pose,
        cancel_requested,
    ):
        recovery_pose.validate()
        for _ in range(self.maximum_recovery_steps):
            current = self._current_probe_pose(
                context_id,
                client_id,
                sensor_id,
            )
            remaining = Vector3Data(
                x=recovery_pose.position.x - current.position.x,
                y=recovery_pose.position.y - current.position.y,
                z=recovery_pose.position.z - current.position.z,
            )
            distance_m = self._norm(remaining)
            if distance_m <= 0.005:
                orientation_error = self._orientation_error(
                    recovery_pose,
                    current,
                )
                if orientation_error > self.maximum_axis_error_rad:
                    return RuntimeError(
                        "Surface recovery returned to the aligned position "
                        "with an excessive orientation error of "
                        f"{math.degrees(orientation_error):.2f} deg"
                    )
                return current

            maximum_component = max(
                abs(remaining.x),
                abs(remaining.y),
                abs(remaining.z),
            )
            scale = min(1.0, self.recovery_step_m / maximum_component)
            translation = Vector3Data(
                x=remaining.x * scale,
                y=remaining.y * scale,
                z=remaining.z * scale,
            )
            result = self._execute_recovery_step(
                context_id,
                client_id,
                request_id,
                translation,
                cancel_requested,
            )
            if result is None or isinstance(result, Exception):
                return result
        return RuntimeError(
            "Surface recovery could not reach the original aligned "
            "pre-approach pose within the recovery step limit"
        )

    def _execute_recovery_step(
        self,
        context_id,
        client_id,
        request_id,
        translation,
        cancel_requested,
    ):
        context = self.coordinator.context(context_id, client_id)
        operation = self.coordinator.prepare_motion(
            context,
            ProbeMotionRequest(
                kind=ProbeMotionKind.ADJUST_PROBE_DISTANCE,
                frame=ProbeMotionFrame.TAG,
                translation=translation,
                position_tolerance_m=0.005,
                orientation_tolerance_rad=0.05,
            ),
            surface_verification_request_id=request_id,
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

        user_cancelled = False
        cancellation_sent = False
        while not waiter.event.wait(self.poll_sec):
            if cancel_requested() and not cancellation_sent:
                user_cancelled = True
                cancellation_sent = True
                self._cancel_motion(
                    context_id,
                    client_id,
                    operation.request_id,
                )
        with self._lock:
            self._waiters.pop(operation.request_id, None)
        if user_cancelled:
            return None
        if (
            waiter.status is None
            or waiter.status.state is not CommandControllerState.SUCCEEDED
        ):
            detail = (
                waiter.status.detail
                if waiter.status is not None
                else "Surface recovery motion ended without a result"
            )
            return RuntimeError(detail)
        return waiter.status

    def _current_probe_pose(
        self,
        context_id,
        client_id,
        sensor_id,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.snapshot(context)
        return self.state_source.current_probe_pose_object(
            snapshot.selected_reference_tag_id,
            sensor_id,
        )

    def _cancel_motion(
        self,
        context_id,
        client_id,
        request_id,
    ):
        try:
            context = self.coordinator.context(context_id, client_id)
            self.coordinator.cancel_motion(context, request_id)
        except LookupError:
            pass

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

    def _abort(
        self,
        context_id,
        client_id,
        request_id,
        exception,
        state_changed,
    ):
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.abort_surface_verification(
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
    def _orientation_error(target_pose, current_pose):
        target = target_pose.orientation
        current = current_pose.orientation
        dot = abs(
            target.x * current.x
            + target.y * current.y
            + target.z * current.z
            + target.w * current.w
        )
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    @staticmethod
    def _norm(vector):
        return math.sqrt(
            vector.x * vector.x
            + vector.y * vector.y
            + vector.z * vector.z
        )

    @staticmethod
    def _positive_integer(value, label):
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f"{label} must be a positive integer")
        return value

    @staticmethod
    def _non_negative_integer(value, label):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{label} must be a non-negative integer")
        return value

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
