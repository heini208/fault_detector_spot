"""Run server-owned probe refinement finalization."""

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from time import monotonic
from threading import Event, RLock

from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)


_TERMINAL_STATES = frozenset({
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
    CommandControllerState.STOP_CONFIRMED,
})


class FinalizationPhase(str, Enum):
    """Observable phases of one finalization transaction."""

    APPROVING = "approving"
    SAVING = "saving"
    RETRACTING_TO_ALIGNED = "retracting_to_aligned"
    RETRACTING_TO_SAFE = "retracting_to_safe"
    COMPLETE = "complete"
    RECOVERY_REQUIRED = "recovery_required"


@dataclass(frozen=True)
class ProbeRefinementFinalizationSpec:
    """Describe whether finalization persists before mandatory retraction."""

    save_requested: bool
    probe_point_id: str = ""
    probe_point_display_name: str = ""
    position_tolerance_m: float = 0.01
    orientation_tolerance_rad: float = 0.087
    measurement_duration_sec: float = 1.0


@dataclass
class _MotionWaiter:
    event: Event = field(default_factory=Event)
    status: object = None


class ProbeRefinementFinalizationCoordinator:
    """Execute approval, persistence, and two-stage retraction."""

    def __init__(
        self,
        coordinator,
        poll_sec: float = 0.05,
        motion_timeout_sec: float = 120.0,
        cancellation_timeout_sec: float = 5.0,
    ):
        for value in (poll_sec, motion_timeout_sec, cancellation_timeout_sec):
            if not isfinite(value) or value <= 0.0:
                raise ValueError("Finalization timeouts must be finite and positive")
        self.coordinator = coordinator
        self.poll_sec = float(poll_sec)
        self.motion_timeout_sec = float(motion_timeout_sec)
        self.cancellation_timeout_sec = float(cancellation_timeout_sec)
        self._lock = RLock()
        self._waiters = {}
        self._shutdown = Event()
        coordinator.add_motion_status_listener(self._receive_motion_status)

    def run(
        self,
        context,
        request_id: str,
        spec: ProbeRefinementFinalizationSpec,
        cancel_requested,
        state_changed=None,
    ):
        """Run one complete finalization transaction."""
        if not isinstance(spec, ProbeRefinementFinalizationSpec):
            raise TypeError("Expected a ProbeRefinementFinalizationSpec")
        if not callable(cancel_requested):
            raise TypeError("Cancellation predicate must be callable")
        if state_changed is not None and not callable(state_changed):
            raise TypeError("State listener must be callable")

        if self._shutdown.is_set():
            raise RuntimeError("Application is shutting down")

        context_id = context.context_id
        client_id = context.client_id
        saved = False
        snapshot = self.coordinator.begin_finalization(
            context,
            request_id,
            spec.save_requested,
        )

        try:
            self._require_running(cancel_requested)
            if spec.save_requested:
                self._emit(
                    state_changed, FinalizationPhase.APPROVING, saved, snapshot,
                )
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.approve_probe_geometry_for_finalization(
                    context, request_id,
                )
                self._require_running(cancel_requested)
                self._emit(
                    state_changed, FinalizationPhase.SAVING, saved, snapshot,
                )
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.save_probe_point_for_finalization(
                    context,
                    request_id,
                    spec.probe_point_id,
                    spec.probe_point_display_name,
                    spec.position_tolerance_m,
                    spec.orientation_tolerance_rad,
                    spec.measurement_duration_sec,
                )
                saved = True

            for phase, kind in (
                (FinalizationPhase.RETRACTING_TO_ALIGNED,
                 ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH),
                (FinalizationPhase.RETRACTING_TO_SAFE,
                 ProbeMotionKind.MOVE_SAFE_APPROACH),
            ):
                self._emit(state_changed, phase, saved, snapshot)
                self._require_running(cancel_requested)
                status = self._execute_retraction(
                    context_id,
                    client_id,
                    request_id,
                    kind,
                    cancel_requested,
                    spec.position_tolerance_m,
                    spec.orientation_tolerance_rad,
                )
                if status.state is not CommandControllerState.SUCCEEDED:
                    raise RuntimeError(status.detail or "Mandatory retraction failed")
                context = self.coordinator.context(context_id, client_id)
                snapshot = self.coordinator.snapshot(context)

            context = self.coordinator.context(context_id, client_id)
            snapshot = self.coordinator.complete_finalization(context, request_id)
        except Exception as exception:
            # A persisted point is durable even when cancellation, disconnection,
            # or retraction fails. Keep its refinement and mandatory recovery.
            snapshot = self._fail(
                context_id, client_id, request_id, str(exception),
            )
            saved = saved or bool(getattr(
                getattr(snapshot, "refinement", None), "saved", False,
            ))
            self._emit(
                state_changed, FinalizationPhase.RECOVERY_REQUIRED, saved, snapshot,
            )
            return snapshot, saved, FinalizationPhase.RECOVERY_REQUIRED

        self._emit(state_changed, FinalizationPhase.COMPLETE, saved, snapshot)
        return snapshot, saved, FinalizationPhase.COMPLETE

    def _require_running(self, cancel_requested):
        if self._shutdown.is_set():
            raise RuntimeError("Application shutdown interrupted retraction")
        if cancel_requested():
            raise RuntimeError("Probe finalization was cancelled; retraction is required")

    def _execute_retraction(
        self,
        context_id,
        client_id,
        finalization_request_id,
        kind,
        cancel_requested,
        position_tolerance_m,
        orientation_tolerance_rad,
    ):
        if self._shutdown.is_set():
            raise RuntimeError("Application is shutting down")
        self._require_running(cancel_requested)
        context = self.coordinator.context(context_id, client_id)
        operation = self.coordinator.prepare_motion(
            context,
            ProbeMotionRequest(
                kind=kind,
                retract_path=True,
                position_tolerance_m=position_tolerance_m,
                orientation_tolerance_rad=orientation_tolerance_rad,
            ),
            finalization_request_id=finalization_request_id,
        )
        waiter = _MotionWaiter()
        with self._lock:
            self._waiters[operation.request_id] = waiter
        try:
            if self._shutdown.is_set() or cancel_requested():
                # Preparation reserves command ownership. Cancel that reservation
                # before submission when close/cancel arrived during preparation.
                self.coordinator.cancel_motion(context, operation.request_id)
                raise RuntimeError(
                    "Probe finalization cancelled during preparation; retraction is required"
                )
            self.coordinator.submit_motion(operation)
            deadline = monotonic() + self.motion_timeout_sec
            cancellation_deadline = None
            while not waiter.event.is_set():
                stopping = self._shutdown.is_set()
                timed_out = monotonic() >= deadline
                if stopping or cancel_requested() or timed_out:
                    if cancellation_deadline is None:
                        # Retain command ownership until its actual terminal
                        # status; this waiter never manufactures a stopped state.
                        cancellation_deadline = (
                            monotonic() + self.cancellation_timeout_sec
                        )
                        self.coordinator.cancel_motion(context, operation.request_id)
                    if stopping:
                        raise RuntimeError("Application shutdown interrupted retraction")
                    if monotonic() >= cancellation_deadline:
                        raise RuntimeError(
                            "Retraction stop unconfirmed after cancellation deadline; "
                            "recovery is required"
                        )
                waiter.event.wait(self.poll_sec)
            if waiter.status is None:
                raise RuntimeError("Retraction completed without a correlated status")
            return waiter.status
        finally:
            with self._lock:
                self._waiters.pop(operation.request_id, None)

    def _fail(self, context_id, client_id, request_id, detail):
        context = self.coordinator.context(context_id, client_id)
        return self.coordinator.fail_finalization(
            context,
            request_id,
            detail,
        )

    def _receive_motion_status(self, status):
        with self._lock:
            waiter = self._waiters.get(status.request_id)
            if waiter is None or waiter.event.is_set():
                return
            waiter.status = status
            if status.state in _TERMINAL_STATES:
                waiter.event.set()

    @staticmethod
    def _emit(listener, phase, saved, snapshot):
        if listener is not None:
            listener(phase, saved, snapshot)

    def request_shutdown(self):
        """Interrupt retraction waits before the ROS executor is drained."""
        self._shutdown.set()

    def close(self):
        """Detach finalization status observation."""
        try:
            self.coordinator.remove_motion_status_listener(
                self._receive_motion_status
            )
        finally:
            with self._lock:
                waiters = tuple(self._waiters.values())
                self._waiters.clear()
            for waiter in waiters:
                waiter.event.set()


__all__ = [
    "FinalizationPhase",
    "ProbeRefinementFinalizationCoordinator",
    "ProbeRefinementFinalizationSpec",
]
