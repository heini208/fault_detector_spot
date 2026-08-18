"""Run server-owned probe refinement finalization."""

from dataclasses import dataclass, field
from enum import Enum
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

    def __init__(self, coordinator, poll_sec: float = 0.05):
        if poll_sec <= 0.0:
            raise ValueError("Finalization polling interval must be positive")
        self.coordinator = coordinator
        self.poll_sec = float(poll_sec)
        self._lock = RLock()
        self._waiters = {}
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

        context_id = context.context_id
        client_id = context.client_id
        saved = False
        snapshot = self.coordinator.begin_finalization(
            context,
            request_id,
            spec.save_requested,
        )

        if cancel_requested():
            snapshot = self._fail(
                context_id,
                client_id,
                request_id,
                "Probe finalization was cancelled before retraction",
            )
            self._emit(
                state_changed,
                FinalizationPhase.RECOVERY_REQUIRED,
                saved,
                snapshot,
            )
            return snapshot, saved, FinalizationPhase.RECOVERY_REQUIRED

        if spec.save_requested:
            self._emit(
                state_changed,
                FinalizationPhase.APPROVING,
                saved,
                snapshot,
            )
            context = self.coordinator.context(context_id, client_id)
            snapshot = (
                self.coordinator.approve_probe_geometry_for_finalization(
                    context,
                    request_id,
                )
            )
            self._emit(
                state_changed,
                FinalizationPhase.SAVING,
                saved,
                snapshot,
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

        phase = FinalizationPhase.RETRACTING_TO_ALIGNED
        self._emit(state_changed, phase, saved, snapshot)
        status = self._execute_retraction(
            context_id,
            client_id,
            request_id,
            ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
            cancel_requested,
        )
        if status.state is not CommandControllerState.SUCCEEDED:
            detail = status.detail or "Aligned retraction failed"
            snapshot = self._fail(
                context_id,
                client_id,
                request_id,
                detail,
            )
            self._emit(
                state_changed,
                FinalizationPhase.RECOVERY_REQUIRED,
                saved,
                snapshot,
            )
            return snapshot, saved, FinalizationPhase.RECOVERY_REQUIRED

        phase = FinalizationPhase.RETRACTING_TO_SAFE
        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.snapshot(context)
        self._emit(state_changed, phase, saved, snapshot)
        status = self._execute_retraction(
            context_id,
            client_id,
            request_id,
            ProbeMotionKind.MOVE_SAFE_APPROACH,
            cancel_requested,
        )
        if status.state is not CommandControllerState.SUCCEEDED:
            detail = status.detail or "Safe retraction failed"
            snapshot = self._fail(
                context_id,
                client_id,
                request_id,
                detail,
            )
            self._emit(
                state_changed,
                FinalizationPhase.RECOVERY_REQUIRED,
                saved,
                snapshot,
            )
            return snapshot, saved, FinalizationPhase.RECOVERY_REQUIRED

        context = self.coordinator.context(context_id, client_id)
        snapshot = self.coordinator.complete_finalization(
            context,
            request_id,
        )
        self._emit(
            state_changed,
            FinalizationPhase.COMPLETE,
            saved,
            snapshot,
        )
        return snapshot, saved, FinalizationPhase.COMPLETE

    def _execute_retraction(
        self,
        context_id,
        client_id,
        finalization_request_id,
        kind,
        cancel_requested,
    ):
        context = self.coordinator.context(context_id, client_id)
        operation = self.coordinator.prepare_motion(
            context,
            ProbeMotionRequest(kind=kind),
            finalization_request_id=finalization_request_id,
        )
        waiter = _MotionWaiter()
        with self._lock:
            self._waiters[operation.request_id] = waiter
        try:
            self.coordinator.submit_motion(operation)
        except Exception:
            with self._lock:
                self._waiters.pop(operation.request_id, None)
            raise

        cancellation_sent = False
        while not waiter.event.wait(self.poll_sec):
            if cancel_requested() and not cancellation_sent:
                cancellation_sent = True
                try:
                    context = self.coordinator.context(
                        context_id,
                        client_id,
                    )
                    self.coordinator.cancel_motion(
                        context,
                        operation.request_id,
                    )
                except LookupError:
                    pass
        with self._lock:
            self._waiters.pop(operation.request_id, None)
        if waiter.status is None:
            raise RuntimeError(
                "Retraction completed without a correlated status"
            )
        return waiter.status

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
            if waiter is None:
                return
            waiter.status = status
            if status.state in _TERMINAL_STATES:
                waiter.event.set()

    @staticmethod
    def _emit(listener, phase, saved, snapshot):
        if listener is not None:
            listener(phase, saved, snapshot)

    def close(self):
        """Detach finalization status observation."""
        self.coordinator.remove_motion_status_listener(
            self._receive_motion_status
        )
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
