"""Execute server-owned closed-loop probe surface verification."""

import time
from dataclasses import dataclass, field
from threading import Event, RLock

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
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
    """Run sampling and bounded corrections outside the ROS transport layer."""

    def __init__(
        self,
        coordinator,
        state_source,
        settle_sec: float = 0.5,
        sample_timeout_sec: float = 2.0,
        poll_sec: float = 0.05,
    ):
        if state_source is None:
            raise ValueError("Probe surface verification needs a state source")
        self.coordinator = coordinator
        self.state_source = state_source
        self.settle_sec = float(settle_sec)
        self.sample_timeout_sec = float(sample_timeout_sec)
        self.poll_sec = float(poll_sec)
        for value, label in (
            (self.settle_sec, "Surface settle time"),
            (self.sample_timeout_sec, "Surface sample timeout"),
            (self.poll_sec, "Surface workflow polling interval"),
        ):
            if value <= 0.0:
                raise ValueError(f"{label} must be positive")
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
        """Run until convergence, failure, cancellation, or recovery."""
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

        while True:
            context = self.coordinator.context(context_id, client_id)
            snapshot = self.coordinator.snapshot(context)
            verification = snapshot.surface_verification
            if verification is None:
                raise RuntimeError("Surface verification state disappeared")
            if not verification.active:
                return snapshot
            if cancel_requested():
                snapshot = self.coordinator.cancel_surface_verification(
                    context,
                    request_id,
                )
                self._emit(state_changed, snapshot)
                return snapshot

            evaluation = self._wait_for_evaluation(
                snapshot,
                request_id,
                receipt_not_before,
                cancel_requested,
            )
            if evaluation is None:
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.cancel_surface_verification(
                    context,
                    request_id,
                )
                self._emit(state_changed, snapshot)
                return snapshot
            if isinstance(evaluation, Exception):
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.fail_surface_sampling(
                    context,
                    request_id,
                    str(evaluation),
                )
                self._emit(state_changed, snapshot)
                return snapshot

            decision, snapshot = evaluation
            self._emit(state_changed, snapshot)
            verification = snapshot.surface_verification
            if verification.state is SurfaceVerificationState.CONVERGED:
                return snapshot
            if not verification.active:
                return snapshot
            if decision.resample_required:
                receipt_not_before = time.monotonic()
                continue

            operation = self._prepare_correction(
                snapshot,
                decision.correction_m,
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
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.fail_surface_correction(
                    context,
                    request_id,
                    str(exception),
                )
                self._emit(state_changed, snapshot)
                return snapshot

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
            context = self.coordinator.context(context_id, client_id)
            if cancellation_sent or (
                status is not None
                and status.state is CommandControllerState.CANCELLED
            ):
                snapshot = self.coordinator.cancel_surface_verification(
                    context,
                    request_id,
                )
                self._emit(state_changed, snapshot)
                return snapshot
            if (
                status is None
                or status.state is not CommandControllerState.SUCCEEDED
            ):
                detail = (
                    status.detail
                    if status is not None
                    else "Surface correction ended without a result"
                )
                snapshot = self.coordinator.fail_surface_correction(
                    context,
                    request_id,
                    detail,
                )
                self._emit(state_changed, snapshot)
                return snapshot

            snapshot = self.coordinator.mark_surface_correction_succeeded(
                context,
                request_id,
            )
            self._emit(state_changed, snapshot)
            if not self._settle(cancel_requested):
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.cancel_surface_verification(
                    context,
                    request_id,
                )
                self._emit(state_changed, snapshot)
                return snapshot
            context = self.coordinator.context(context_id, client_id)
            snapshot = self.coordinator.resume_surface_sampling(
                context,
                request_id,
            )
            self._emit(state_changed, snapshot)
            receipt_not_before = time.monotonic()

    def _wait_for_evaluation(
        self,
        snapshot,
        request_id,
        receipt_not_before,
        cancel_requested,
    ):
        deadline = time.monotonic() + self.sample_timeout_sec
        last_error = None
        while True:
            if cancel_requested():
                return None
            try:
                samples = self.state_source.surface_distance_samples(
                    snapshot.selected_sensor_id,
                    receipt_not_before=receipt_not_before,
                )
                achieved = self.state_source.current_probe_pose_object(
                    snapshot.selected_reference_tag_id,
                    snapshot.selected_sensor_id,
                )
                context = self.coordinator.context(
                    snapshot.context.context_id,
                    snapshot.context.client_id,
                )
                return self.coordinator.evaluate_surface_verification(
                    context,
                    request_id,
                    samples,
                    achieved,
                )
            except Exception as exception:
                last_error = exception
            if time.monotonic() >= deadline:
                return last_error or RuntimeError(
                    "Timed out waiting for a valid surface sample window"
                )
            time.sleep(self.poll_sec)

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
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.10,
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
        """Detach workflow status observation."""
        self.coordinator.remove_motion_status_listener(
            self._receive_motion_status
        )
        with self._lock:
            waiters = tuple(self._waiters.values())
            self._waiters.clear()
        for waiter in waiters:
            waiter.event.set()


__all__ = ["ProbeSurfaceVerificationRunner"]
