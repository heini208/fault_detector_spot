"""Serialize semantic commands independently of ROS transport."""

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
from math import isfinite
from threading import RLock
import time
from typing import Callable, Deque, List, Optional, Tuple

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_lifecycle import CommandLifecycleConfig
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)


class CommandControllerState(str, Enum):
    """Lifecycle state of one semantic command request."""

    QUEUED = "queued"
    DISPATCHED = "dispatched"
    RUNNING = "running"
    UNKNOWN = "unknown"
    STOP_REQUESTED = "stop_requested"
    STOP_CONFIRMED = "stop_confirmed"
    STOP_UNCONFIRMED = "stop_unconfirmed"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class CommandControllerStatus:
    """Controller-owned state for one correlated semantic command."""

    request: CommandRequest[SemanticCommand]
    state: CommandControllerState
    detail: str = ""
    buffered_command_count: int = 0
    retry_safe: bool = False
    continuation_safe: bool = False
    retry_command: SemanticCommand | None = None
    execution_idle: bool = False
    execution_instance_id: str = ""
    status_sequence: int = 0

    @property
    def request_id(self) -> str:
        return self.request.request_id


@dataclass(frozen=True)
class CommandExecutionStatus:
    """Transport-independent execution feedback for one dispatched request."""

    request_id: str
    state: CommandControllerState
    detail: str = ""
    buffered_command_count: int = 0
    retry_safe: bool = False
    continuation_safe: bool = False
    retry_command: SemanticCommand | None = None
    execution_idle: bool = False
    execution_instance_id: str = ""
    status_sequence: int = 0
    software_stop_latched: bool = False
    software_stop_confirmed: bool = False


@dataclass(frozen=True)
class SoftwareStopStatus:
    """Application-owned latch; confirmation aggregates all execution owners."""

    request_id: str = ""
    latched: bool = False
    stop_confirmed: bool = False
    detail: str = "Software stop is clear"
    status_sequence: int = 0


class DuplicateCommandRequest(ValueError):
    """Raised when a request identity has already been accepted."""


class CommandExecutionUnresolved(ValueError):
    """Normal admission is blocked until the retained execution is reconciled."""


class SoftwareStopLatched(CommandExecutionUnresolved):
    """Operator reset is required even after all owners confirm stopping."""


class UnknownCommandRequest(LookupError):
    """Raised when cancellation targets an unknown request."""


StatusListener = Callable[[CommandControllerStatus], None]
AcceptedListener = Callable[[CommandRequest[SemanticCommand]], None]
DispatchCallback = Callable[[CommandRequest[SemanticCommand]], None]
DispatchReady = Callable[[], bool]
ListenerErrorHandler = Callable[[Exception], None]
RequestPreparer = Callable[
    [CommandRequest[SemanticCommand]],
    CommandRequest[SemanticCommand],
]


class CommandController:
    """Own the single semantic command queue and result correlation."""

    def __init__(
        self,
        dispatch_request: Optional[DispatchCallback] = None,
        dispatch_ready: Optional[DispatchReady] = None,
        monotonic_clock: Callable[[], float] = time.monotonic,
        ack_timeout_sec: float = 3.0,
        listener_error_handler: Optional[ListenerErrorHandler] = None,
        heartbeat_timeout_sec: float = 5.0,
        reconciliation_interval_sec: float = 1.0,
        session_work_changed=None,
    ):
        if dispatch_request is not None and not callable(dispatch_request):
            raise TypeError("Dispatch callback must be callable")
        if dispatch_ready is not None and not callable(dispatch_ready):
            raise TypeError("Dispatch readiness callback must be callable")
        if session_work_changed is not None and not callable(session_work_changed):
            raise TypeError("Session ownership callback must be callable")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        limits = CommandLifecycleConfig(
            ack_timeout_sec=ack_timeout_sec, heartbeat_timeout_sec=heartbeat_timeout_sec,
            reconciliation_interval_sec=reconciliation_interval_sec,
        )
        if (
            listener_error_handler is not None
            and not callable(listener_error_handler)
        ):
            raise TypeError("Listener error handler must be callable")

        self._lock = RLock()
        # Never acquire the command/setup lock while holding this transport lock.
        self._emergency_lock = RLock()
        self._pending_emergencies = deque()
        self._emergency_feedback = {}
        self._emergency_request_ids = set()
        self._unresolved_dispatch_ids = set()
        self._admission_closed = False
        self._session_work_changed = session_work_changed
        self._session_work_unresolved = False
        self._external_session_work = False
        self._stops = {}
        self._active_state = None
        self._reconcile_request = None
        self._last_reconciled = {}
        self._request_execution_instances = {}
        self._retired_execution_instances = set()
        self._last_execution_receipt = {}
        self._last_execution_sequence = {}
        self._public_status_sequence = 0
        self._last_queue_status = {}
        self._current_cancel_request_id = None
        self._software_stop = SoftwareStopStatus(status_sequence=1)
        self._software_stop_listeners = []
        self._last_stop_publication = monotonic_clock()
        self._owner_stop_confirmed = False
        self._stop_participant = None
        self._participant_stop_confirmed = True
        self._participant_stop_detail = ""
        self._reset_request_id = None
        self._queue: Deque[CommandRequest[SemanticCommand]] = deque()
        self._active: Optional[CommandRequest[SemanticCommand]] = None
        self._listeners: List[StatusListener] = []
        self._accepted_listeners: List[AcceptedListener] = []
        self._request_preparers: List[RequestPreparer] = []
        self._known_request_ids = set()
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._ack_timeout_sec = limits.ack_timeout_sec
        self.lifecycle_limits = limits
        self._heartbeat_timeout_sec = limits.heartbeat_timeout_sec
        self._reconciliation_interval_sec = limits.reconciliation_interval_sec
        self._monotonic_clock = monotonic_clock
        self._dispatch_request = dispatch_request
        self._dispatch_ready = dispatch_ready
        self._dispatch_emergency_ready = dispatch_ready
        self._dispatch_reset_ready = dispatch_ready
        self._configuration_ready = None
        self._listener_error_handler = listener_error_handler

    @property
    def active_request_id(self) -> str:
        with self._lock:
            if self._active is not None:
                return self._active.request_id
            if self._stops:
                return next(iter(self._stops))
            with self._emergency_lock:
                return self._pending_emergencies[0][0].request_id if self._pending_emergencies else ""

    @property
    def queued_request_ids(self) -> Tuple[str, ...]:
        with self._lock:
            return tuple(request.request_id for request in self._queue)

    def knows_request(self, request_id: str) -> bool:
        """Prevent admission rejection from impersonating execution feedback."""
        with self._emergency_lock:
            return request_id in self._known_request_ids or request_id in self._emergency_request_ids

    @property
    def software_stop_status(self):
        with self._emergency_lock:
            return self._software_stop

    def add_software_stop_listener(self, listener):
        """Listeners must be nonblocking; stop admission bypasses workflow locks."""
        if not callable(listener):
            raise TypeError("Software stop listener must be callable")
        with self._emergency_lock:
            if listener not in self._software_stop_listeners:
                self._software_stop_listeners.append(listener)
            self._call_listener(listener, self._software_stop)

    def remove_software_stop_listener(self, listener):
        with self._emergency_lock:
            if listener in self._software_stop_listeners:
                self._software_stop_listeners.remove(listener)

    def configure_stop_participant(self, request_stop, query_status, *, reset=None):
        """Include the existing acquisition owner, including an ongoing recording."""
        if not callable(request_stop) or not callable(query_status) or (reset is not None and not callable(reset)):
            raise TypeError("Stop participant callbacks must be callable")
        with self._emergency_lock:
            self._stop_participant = (request_stop, query_status, reset)
            if self._software_stop.latched:
                self._participant_stop_confirmed = False
                request_stop(self._current_cancel_request_id)

    def observe_owner_software_stop(self, latched, confirmed=None):
        """Reconnecting to a latched execution process cannot silently clear it."""
        if latched and not self.software_stop_status.latched:
            self.cancel_all(client_id="execution_owner_reconciliation")
        if confirmed is False:
            with self._lock:
                self._owner_stop_confirmed = False
                for stop in self._stops.values():
                    stop["owner_confirmed"] = False
                self._refresh_stop_confirmation_locked()

    def _publish_software_stop_locked(self, *, latched=None, confirmed=None, detail=None):
        previous = self._software_stop
        self._software_stop = SoftwareStopStatus(
            request_id=self._current_cancel_request_id or previous.request_id,
            latched=previous.latched if latched is None else latched,
            stop_confirmed=previous.stop_confirmed if confirmed is None else confirmed,
            detail=previous.detail if detail is None else detail,
            status_sequence=previous.status_sequence + 1,
        )
        self._last_stop_publication = self._monotonic_clock()
        for listener in tuple(self._software_stop_listeners):
            self._call_listener(listener, self._software_stop)

    def reset_software_stop(self, client_id="command_controller"):
        """Request a separately acknowledged, nonmoving owner reset."""
        return self.submit(CommandRequest.create(
            SemanticCommand(command_id=CommandID.RESET_SOFTWARE_STOP),
            client_id=client_id, origin=CommandOrigin.SYSTEM,
            recording_policy=RecordingPolicy.EXCLUDE,
        ))

    def bind_execution_owner(self, request_id: str, execution_instance_id: str) -> None:
        """Bind before submission; do not take the setup lock on the stop path."""
        if not isinstance(execution_instance_id, str) or not execution_instance_id.strip():
            raise ValueError("Execution instance identity is required")
        with self._emergency_lock:
            previous = self._request_execution_instances.get(request_id)
            if previous is not None and previous != execution_instance_id:
                raise ValueError("An unresolved request cannot change execution instance")
            if execution_instance_id in self._retired_execution_instances:
                raise ValueError("Execution instance has retired")
            self._request_execution_instances[request_id] = execution_instance_id

    def execution_owner_changed(self, previous_instance: str, current_instance: str) -> None:
        """A freshly queried replacement cannot inherit unresolved execution."""
        if not previous_instance or previous_instance == current_instance:
            return
        with self._lock:
            with self._emergency_lock:
                self._retired_execution_instances.add(previous_instance)
                if self._software_stop.latched:
                    self._owner_stop_confirmed = False
                    self._publish_software_stop_locked(confirmed=False,
                        detail="Execution owner restarted; prior stop evidence is unresolved")
            self._reconcile_emergencies_locked()
            detail = (f"Execution owner changed from {previous_instance} to {current_instance}; "
                      "previous ownership is unresolved, motion admission remains blocked")
            if self._active is not None and self._request_execution_instances.get(self._active.request_id) == previous_instance:
                self._active_state = (CommandControllerState.STOP_UNCONFIRMED
                                      if self._active_state in {CommandControllerState.STOP_REQUESTED,
                                                                CommandControllerState.STOP_UNCONFIRMED}
                                      else CommandControllerState.UNKNOWN)
                self._emit_locked(self._active, self._active_state, detail)
            for stop in self._stops.values():
                if self._request_execution_instances.get(stop["request"].request_id) == previous_instance:
                    stop["owner_confirmed"] = False
                    stop["state"] = CommandControllerState.STOP_UNCONFIRMED
                    self._emit_locked(stop["request"], stop["state"], detail)

    def configure_dispatch(
        self,
        dispatch_request: DispatchCallback,
        dispatch_ready: Optional[DispatchReady] = None,
        emergency_ready: Optional[DispatchReady] = None,
        reset_ready: Optional[DispatchReady] = None,
        configuration_ready: Optional[DispatchReady] = None,
    ) -> None:
        """Attach the execution transport used for future dispatches."""
        if not callable(dispatch_request):
            raise TypeError("Dispatch callback must be callable")
        if dispatch_ready is not None and not callable(dispatch_ready):
            raise TypeError("Dispatch readiness callback must be callable")
        if emergency_ready is not None and not callable(emergency_ready):
            raise TypeError("Emergency readiness callback must be callable")
        if reset_ready is not None and not callable(reset_ready):
            raise TypeError("Reset readiness callback must be callable")
        if configuration_ready is not None and not callable(configuration_ready):
            raise TypeError("Configuration readiness callback must be callable")
        with self._lock:
            self._dispatch_request = dispatch_request
            self._dispatch_ready = dispatch_ready
            self._dispatch_emergency_ready = emergency_ready if emergency_ready is not None else dispatch_ready
            self._dispatch_reset_ready = reset_ready if reset_ready is not None else dispatch_ready
            self._configuration_ready = configuration_ready
            self._dispatch_next_locked()

    def configure_reconciliation(self, query_status) -> None:
        """Attach a read-only, nonblocking execution-owner status query."""
        if query_status is not None and not callable(query_status):
            raise TypeError("Status query must be callable")
        with self._lock:
            self._reconcile_request = query_status

    def clear_dispatch(self) -> None:
        """Detach the execution transport without changing queued requests."""
        with self._lock:
            self._dispatch_request = None
            self._dispatch_ready = None
            self._dispatch_emergency_ready = None
            self._dispatch_reset_ready = None
            if self._configuration_ready is not None:
                self._configuration_ready = lambda: False
            self._reconcile_request = None

    def add_status_listener(self, listener: StatusListener) -> None:
        if not callable(listener):
            raise TypeError("Status listener must be callable")
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_status_listener(self, listener: StatusListener) -> None:
        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def add_accepted_listener(self, listener: AcceptedListener) -> None:
        if not callable(listener):
            raise TypeError("Accepted listener must be callable")
        with self._lock:
            if listener not in self._accepted_listeners:
                self._accepted_listeners.append(listener)

    def remove_accepted_listener(self, listener: AcceptedListener) -> None:
        with self._lock:
            if listener in self._accepted_listeners:
                self._accepted_listeners.remove(listener)

    def add_request_preparer(self, preparer: RequestPreparer) -> None:
        """Register one atomic command-admission preparation step."""
        if not callable(preparer):
            raise TypeError("Request preparer must be callable")
        with self._lock:
            if preparer not in self._request_preparers:
                self._request_preparers.append(preparer)

    def remove_request_preparer(self, preparer: RequestPreparer) -> None:
        """Remove one command-admission preparation step."""
        with self._lock:
            if preparer in self._request_preparers:
                self._request_preparers.remove(preparer)

    def run_if_idle(
        self,
        action,
        busy_message: str = "Physical commands are active or queued",
    ):
        """Run one shared-state mutation atomically while the lane is idle."""
        return self._run_idle_action(action, busy_message, configuration_only=False)

    def close_admission(self):
        """Fence future dispatch and discard never-dispatched work without motion."""
        with self._emergency_lock:
            self._admission_closed = True
        with self._lock:
            queued = list(self._queue)
            self._queue.clear()
            with self._emergency_lock:
                if self._active is not None and self._active.request_id not in self._unresolved_dispatch_ids:
                    queued.insert(0, self._active)
                    self._active = None
                    self._active_state = None
                    self._active_acknowledged = False
                    self._active_dispatched_monotonic = None
                if any(request.request_id == self._reset_request_id for request in queued):
                    self._reset_request_id = None
            for request in queued:
                self._emit_locked(request, CommandControllerState.CANCELLED,
                                  "Cancelled before dispatch by application shutdown", execution_idle=True)

    def shutdown_status(self):
        """Confirm closed local ownership without inventing a global robot stop."""
        with self._lock:
            with self._emergency_lock:
                idle = bool(self._admission_closed and self._active is None and not self._queue
                            and not self._unresolved_dispatch_ids and not self._stops
                            and not self._pending_emergencies and not self._software_stop.latched)
            return idle, ("Command admission closed; no dispatched work remains" if idle
                          else "Command execution or an existing software stop remains unresolved")

    def set_external_session_work(self, unresolved):
        """Include an acquisition that may outlive its start command."""
        with self._emergency_lock:
            self._external_session_work = bool(unresolved)
            self._refresh_session_work_locked()

    def _set_session_work_locked(self, unresolved):
        if unresolved == self._session_work_unresolved:
            return
        if self._session_work_changed is not None:
            try:
                self._session_work_changed(unresolved)
            except Exception:
                # Atomic replacement can succeed before directory fsync fails.
                # Neither the old nor requested value is then proven durable;
                # the next dispatch must write dirty ownership again.
                self._session_work_unresolved = None
                raise
        self._session_work_unresolved = unresolved

    def _refresh_session_work_locked(self):
        unresolved = bool(self._unresolved_dispatch_ids or self._software_stop.latched
                          or self._external_session_work)
        try:
            self._set_session_work_locked(unresolved)
        except Exception as exception:
            # Failed cleanup persistence retains the previous dirty evidence.
            self._handle_listener_error(exception)

    def run_configuration_change(
        self,
        action,
        busy_message: str = "Physical commands are active or queued",
    ):
        """Permit nonmoving configuration during a stop with no owned motion.

        Stop bookkeeping is not a geometry-dependent command. Configuration
        must not clear that bookkeeping, reopen motion, or start a workflow.
        """
        return self._run_idle_action(action, busy_message, configuration_only=True)

    def _run_idle_action(self, action, busy_message, *, configuration_only):
        if not callable(action):
            raise TypeError("Idle-lane action must be callable")
        if not isinstance(busy_message, str) or not busy_message.strip():
            raise ValueError("Busy-lane message must not be empty")
        with self._lock:
            with self._emergency_lock:
                if self._admission_closed:
                    raise RuntimeError("Application shutdown has closed command admission")
                unresolved_motion = self._unresolved_dispatch_ids - self._emergency_request_ids
                stop_pending = bool(self._stops or self._pending_emergencies
                    or (self._software_stop.latched and not self._software_stop.stop_confirmed))
                configuration_allowed = bool(configuration_only and self._software_stop.latched
                                             and self._participant_stop_confirmed)
            if self._active is not None or self._queue or unresolved_motion:
                raise RuntimeError(busy_message.strip())
            if stop_pending:
                if not configuration_only:
                    raise RuntimeError(
                        "Software stop is pending or unconfirmed; this operation requires "
                        f"confirmed stopping. {self._software_stop.detail}"
                    )
                if not self._participant_stop_confirmed:
                    raise RuntimeError(
                        "Sensor acquisition stop is not confirmed. "
                        + self._participant_stop_detail
                    )
                if configuration_allowed and self._configuration_ready is not None:
                    try:
                        configuration_allowed = bool(self._configuration_ready())
                    except Exception as exception:
                        self._handle_listener_error(exception)
                        configuration_allowed = False
                if not configuration_allowed:
                    raise RuntimeError(
                        "Execution-owner status is unavailable or still reports "
                        "unresolved commands; configuration must wait"
                    )
            return action()

    def submit(self, request: CommandRequest[SemanticCommand]) -> str:
        normalized = self._normalize_request(request)
        if self._is_emergency(normalized):
            return self._submit_emergency(normalized)
        resetting = normalized.command.command_id is CommandID.RESET_SOFTWARE_STOP
        with self._emergency_lock:
            if self._admission_closed:
                raise RuntimeError("Application shutdown has closed command admission")
            if self._software_stop.latched and not resetting:
                raise SoftwareStopLatched("Software stop is latched; explicit reset is required before motion")
        with self._lock:
            if resetting:
                self._refresh_stop_confirmation_locked()
                with self._emergency_lock:
                    if self._reset_request_id is not None:
                        return self._reset_request_id
                    if not self._software_stop.latched:
                        raise ValueError("Software stop is already clear")
                    if not self._software_stop.stop_confirmed or self._active is not None or self._stops or self._pending_emergencies:
                        raise CommandExecutionUnresolved("Software stop reset requires confirmed stopping from every owner")
                    if normalized.request_id in self._known_request_ids or normalized.request_id in self._emergency_request_ids:
                        raise DuplicateCommandRequest(f"Duplicate request ID: {normalized.request_id}")
                    # Reserve while validating, so a concurrent stop can veto
                    # this reset even before ordinary admission callbacks run.
                    self._reset_request_id = normalized.request_id
            if self._active_state in {CommandControllerState.UNKNOWN, CommandControllerState.STOP_UNCONFIRMED} or any(
                stop["state"] is CommandControllerState.STOP_UNCONFIRMED for stop in self._stops.values()
            ):
                raise CommandExecutionUnresolved("Execution ownership is unresolved; normal commands are not admitted")
            if not resetting:
                normalized = self._prepare_request_locked(normalized)
            with self._emergency_lock:
                if self._admission_closed:
                    raise RuntimeError("Application shutdown interrupted command admission")
                if self._software_stop.latched and not resetting:
                    raise SoftwareStopLatched("Software stop interrupted command admission")
                if resetting and self._reset_request_id != normalized.request_id:
                    raise SoftwareStopLatched("A newer software stop interrupted reset admission")
                if (
                    normalized.request_id in self._known_request_ids
                    or normalized.request_id in self._emergency_request_ids
                ):
                    raise DuplicateCommandRequest(
                        f"Duplicate request ID: {normalized.request_id}"
                    )
                self._known_request_ids.add(normalized.request_id)
            self._emit_accepted_locked(normalized)
            self._queue.append(normalized)
            self._emit_locked(
                normalized,
                CommandControllerState.QUEUED,
                self._queued_detail_locked(normalized),
            )
            self._dispatch_next_locked()
        return normalized.request_id

    def cancel(self, request_id: str) -> str:
        normalized_id = validate_request_id(request_id)
        with self._lock:
            for request in tuple(self._queue):
                if request.request_id == normalized_id:
                    self._queue.remove(request)
                    with self._emergency_lock:
                        if request.request_id == self._reset_request_id:
                            self._reset_request_id = None
                    self._emit_locked(
                        request,
                        CommandControllerState.CANCELLED,
                        "Cancelled before dispatch",
                        execution_idle=True,
                    )
                    return request.request_id
            if (
                self._active is None
                or self._active.request_id != normalized_id
            ):
                raise UnknownCommandRequest(
                    f"Unknown request ID: {normalized_id}"
                )
        return self.cancel_all()

    def cancel_all(
        self,
        client_id: str = "command_controller",
    ) -> str:
        command = SemanticCommand(command_id=CommandID.EMERGENCY_CANCEL)
        request = CommandRequest.create(
            command=command,
            client_id=client_id,
            origin=CommandOrigin.SYSTEM,
            recording_policy=RecordingPolicy.EXCLUDE,
        )
        return self.submit(request)

    def handle_execution_status(self, status: CommandExecutionStatus, *, receipt_monotonic=None) -> bool:
        """Apply correlated owner evidence, never infer stopping from silence."""
        received_at = self._monotonic_clock()
        if receipt_monotonic is not None:
            if isinstance(receipt_monotonic, bool) or not isfinite(receipt_monotonic):
                raise ValueError("Receipt time must be a finite monotonic reading")
            received_at = min(received_at, receipt_monotonic)
        if not isinstance(status, CommandExecutionStatus):
            raise TypeError("Expected a CommandExecutionStatus")
        if status.state not in {
            CommandControllerState.RUNNING, CommandControllerState.UNKNOWN,
            CommandControllerState.SUCCEEDED, CommandControllerState.FAILED,
            CommandControllerState.CANCELLED, CommandControllerState.STOP_REQUESTED,
            CommandControllerState.STOP_CONFIRMED, CommandControllerState.STOP_UNCONFIRMED,
        }:
            return False
        if (not isinstance(status.execution_instance_id, str) or not status.execution_instance_id
                or not isinstance(status.status_sequence, int) or isinstance(status.status_sequence, bool)
                or status.status_sequence <= 0):
            return False
        with self._emergency_lock:
            feedback = self._emergency_feedback.get(status.request_id)
            if feedback is not None:
                feedback.append((status, received_at))
                return True
        with self._lock:
            self._reconcile_emergencies_locked()
            if status.request_id not in self._stops and (
                self._active is None or status.request_id != self._active.request_id
            ) and not (self._software_stop.latched and status.request_id == self._current_cancel_request_id):
                return False
            with self._emergency_lock:
                expected = self._request_execution_instances.get(status.request_id)
                if status.execution_instance_id in self._retired_execution_instances or (
                    expected is not None and status.execution_instance_id != expected
                ):
                    return False
                if expected is None:
                    self._request_execution_instances[status.request_id] = status.execution_instance_id
            if status.status_sequence <= self._last_execution_sequence.get(status.request_id, 0):
                return False
            self._last_execution_sequence[status.request_id] = status.status_sequence
            self._last_execution_receipt[status.request_id] = received_at
            expired = self._monotonic_clock() - received_at >= self._heartbeat_timeout_sec
            stop = self._stops.get(status.request_id)
            if stop is not None:
                stop["acknowledged"] = True
                confirmed = (
                    status.state is CommandControllerState.STOP_CONFIRMED
                    and status.execution_idle and status.buffered_command_count == 0
                    and not expired
                )
                if confirmed:
                    stop["owner_confirmed"] = True
                    self._owner_stop_confirmed = True
                    self._refresh_stop_confirmation_locked()
                    self._dispatch_next_locked()
                else:
                    stop["owner_confirmed"] = False
                    self._owner_stop_confirmed = False
                    state = (CommandControllerState.STOP_REQUESTED
                             if status.state in {CommandControllerState.RUNNING,
                                                 CommandControllerState.STOP_REQUESTED}
                             and not expired
                             and stop["state"] is not CommandControllerState.STOP_UNCONFIRMED
                             else CommandControllerState.STOP_UNCONFIRMED)
                    stop["state"] = state
                    self._emit_locked(stop["request"], state, status.detail or
                                      "Stop is not yet confirmed by the execution owner")
                    self._refresh_stop_confirmation_locked()
                return True
            if self._software_stop.latched and status.request_id == self._current_cancel_request_id:
                # Keep querying the confirmed stop while latched. Its proof can
                # be revoked by a new owner observation or loss of liveness.
                self._owner_stop_confirmed = bool(
                    status.state is CommandControllerState.STOP_CONFIRMED
                    and status.execution_idle and status.buffered_command_count == 0
                    and status.software_stop_latched and status.software_stop_confirmed
                    and not expired
                )
                self._refresh_stop_confirmation_locked()
                return True
            if self._active is None or status.request_id != self._active.request_id:
                return False
            request = self._active
            self._active_acknowledged = True
            stopping = self._active_state in {
                CommandControllerState.STOP_REQUESTED, CommandControllerState.STOP_UNCONFIRMED,
            }
            terminal = status.state in {
                CommandControllerState.SUCCEEDED, CommandControllerState.FAILED,
                CommandControllerState.CANCELLED, CommandControllerState.STOP_CONFIRMED,
            }
            if terminal and status.execution_idle and status.buffered_command_count == 0 and (
                not stopping or status.state is CommandControllerState.STOP_CONFIRMED
            ):
                if request.command.command_id is CommandID.RESET_SOFTWARE_STOP:
                    with self._emergency_lock:
                        reset_succeeded = (
                            status.state is CommandControllerState.SUCCEEDED
                            and request.request_id == self._reset_request_id
                            and not status.software_stop_latched and status.software_stop_confirmed
                            and self._participant_stop_confirmed and not self._pending_emergencies
                        )
                        if reset_succeeded and self._stop_participant is not None and self._stop_participant[2] is not None:
                            try:
                                self._stop_participant[2]()
                            except Exception as exception:
                                reset_succeeded = False
                                self._participant_stop_confirmed = False
                                self._participant_stop_detail = f"Acquisition reset is unconfirmed: {exception}"
                        if reset_succeeded:
                            previous_stop = self._current_cancel_request_id
                            self._current_cancel_request_id = None
                            self._forget_dispatch_locked(previous_stop)
                            self._publish_software_stop_locked(latched=False, confirmed=True,
                                detail="Software stop reset confirmed; no motion was started")
                        elif status.state is CommandControllerState.SUCCEEDED:
                            status = replace(status, state=CommandControllerState.FAILED,
                                detail="Software stop remains latched: reset proof is incomplete")
                        self._reset_request_id = None
                self._active = None
                self._active_state = None
                self._active_acknowledged = False
                self._active_dispatched_monotonic = None
                self._forget_dispatch_locked(status.request_id)
                self._emit_locked(
                    request, status.state, status.detail, execution_idle=True,
                    retry_safe=status.state is CommandControllerState.FAILED and status.retry_safe,
                    continuation_safe=status.state is CommandControllerState.FAILED and status.continuation_safe,
                    retry_command=status.retry_command if status.state is CommandControllerState.FAILED else None,
                )
                self._refresh_stop_confirmation_locked()
                self._dispatch_next_locked()
                return True
            if stopping:
                state = (CommandControllerState.STOP_UNCONFIRMED if terminal or status.state in {
                    CommandControllerState.UNKNOWN, CommandControllerState.STOP_UNCONFIRMED,
                } else self._active_state)
            elif status.state is CommandControllerState.SUCCEEDED and status.buffered_command_count > 0:
                state = CommandControllerState.RUNNING
            elif terminal:
                state = CommandControllerState.UNKNOWN
            else:
                state = status.state
            if expired and state in {CommandControllerState.RUNNING, CommandControllerState.STOP_REQUESTED}:
                state = (CommandControllerState.STOP_UNCONFIRMED if stopping
                         else CommandControllerState.UNKNOWN)
            self._active_state = state
            detail = status.detail
            if expired:
                detail += "; owner observation is no longer live, ownership remains unresolved"
            if terminal and state in {CommandControllerState.UNKNOWN, CommandControllerState.STOP_UNCONFIRMED}:
                detail += "; execution owner has not confirmed this request stopped"
            self._emit_locked(request, state, detail)
            return True

    def poll(self) -> None:
        """Reconcile retained requests without ever repeating uncertain motion."""
        with self._lock:
            self._reconcile_emergencies_locked()
            self._check_acknowledgement_locked()
            self._check_liveness_locked()
            self._poll_stops_locked()
            self._query_owners_locked()
            self._refresh_stop_confirmation_locked()
            self._dispatch_next_locked()
            self._publish_queue_heartbeats_locked()
            with self._emergency_lock:
                self._refresh_session_work_locked()
                if self._monotonic_clock() - self._last_stop_publication >= self._reconciliation_interval_sec:
                    self._publish_software_stop_locked()

    def _publish_queue_heartbeats_locked(self):
        """Confirm queue ownership without claiming that execution has started."""
        now = self._monotonic_clock()
        interval = min(self._reconciliation_interval_sec, self._heartbeat_timeout_sec / 2.0)
        for request in tuple(self._queue):
            if (request in self._queue
                    and now - self._last_queue_status.get(request.request_id, now) >= interval):
                self._emit_locked(request, CommandControllerState.QUEUED,
                                  self._queued_detail_locked(request))

    def _forget_dispatch_locked(self, request_id):
        with self._emergency_lock:
            self._unresolved_dispatch_ids.discard(request_id)
            self._refresh_session_work_locked()
            if self._software_stop.latched and request_id == self._current_cancel_request_id:
                return
        self._last_reconciled.pop(request_id, None)
        self._last_execution_receipt.pop(request_id, None)
        self._last_execution_sequence.pop(request_id, None)

    def _refresh_stop_confirmation_locked(self):
        completed_stops = []
        with self._emergency_lock:
            if not self._software_stop.latched:
                return
            if self._stop_participant is not None:
                try:
                    confirmed, detail = self._stop_participant[1](self._current_cancel_request_id)
                    self._participant_stop_confirmed = bool(confirmed)
                    self._participant_stop_detail = detail
                except Exception as exception:
                    self._participant_stop_confirmed = False
                    self._participant_stop_detail = f"Acquisition stopping is unconfirmed: {exception}"
            if self._participant_stop_confirmed:
                for request_id, stop in tuple(self._stops.items()):
                    if stop.get("owner_confirmed"):
                        del self._stops[request_id]
                        self._forget_dispatch_locked(request_id)
                        completed_stops.append(stop["request"])
            confirmed = bool(self._owner_stop_confirmed and self._participant_stop_confirmed
                and not self._stops and not self._pending_emergencies
                and (self._active is None or self._active.request_id == self._reset_request_id))
            detail = ("Stopping confirmed; software stop remains latched until explicit reset" if confirmed
                      else self._participant_stop_detail if not self._participant_stop_confirmed
                      else "Software stop latched; execution ownership or physical stopping is unconfirmed")
            if confirmed != self._software_stop.stop_confirmed or detail != self._software_stop.detail:
                self._publish_software_stop_locked(confirmed=confirmed, detail=detail)
        # Ordinary listeners can acquire workflow locks. Never hold the stop
        # admission lock while delivering their notifications.
        for request in completed_stops:
            self._emit_locked(request, CommandControllerState.STOP_CONFIRMED,
                "Robot and acquisition owners confirmed stopping; software stop remains latched",
                execution_idle=True)

    def _check_liveness_locked(self):
        now = self._monotonic_clock()
        if self._software_stop.latched:
            last = self._last_execution_receipt.get(self._current_cancel_request_id)
            if last is not None and now - last >= self._heartbeat_timeout_sec:
                self._owner_stop_confirmed = False
        if self._active is not None and self._active_acknowledged:
            last = self._last_execution_receipt.get(self._active.request_id)
            if last is not None and now - last >= self._heartbeat_timeout_sec:
                state = (CommandControllerState.STOP_UNCONFIRMED
                         if self._active_state in {CommandControllerState.STOP_REQUESTED,
                                                   CommandControllerState.STOP_UNCONFIRMED}
                         else CommandControllerState.UNKNOWN)
                if self._active_state is not state:
                    self._active_state = state
                    self._emit_locked(self._active, state,
                                      "Execution owner liveness expired; ownership is unresolved and motion admission is blocked")
        for stop in self._stops.values():
            last = self._last_execution_receipt.get(stop["request"].request_id)
            if (last is not None and now - last >= self._heartbeat_timeout_sec
                    and stop["state"] is not CommandControllerState.STOP_UNCONFIRMED):
                stop["owner_confirmed"] = False
                self._owner_stop_confirmed = False
                stop["state"] = CommandControllerState.STOP_UNCONFIRMED
                self._emit_locked(stop["request"], stop["state"],
                                  "Stop owner liveness expired; stopping remains unconfirmed")

    def _query_owners_locked(self):
        if self._reconcile_request is None:
            return
        now = self._monotonic_clock()
        request_ids = list(self._stops)
        if (self._software_stop.latched and self._current_cancel_request_id
                and self._current_cancel_request_id not in request_ids):
            request_ids.append(self._current_cancel_request_id)
        if self._active is not None:
            request_ids.append(self._active.request_id)
        for request_id in request_ids:
            previous = self._last_reconciled.get(request_id, now)
            if now - previous < self._reconciliation_interval_sec:
                continue
            self._last_reconciled[request_id] = now
            try:
                self._reconcile_request(request_id)
            except Exception as exception:
                self._handle_listener_error(exception)

    def _normalize_request(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> CommandRequest[SemanticCommand]:
        if not isinstance(request, CommandRequest):
            raise TypeError("Expected an application CommandRequest")
        if not isinstance(request.command, SemanticCommand):
            raise TypeError(
                "Command request must contain SemanticCommand"
            )
        return CommandRequest(
            request_id=request.request_id,
            client_id=request.client_id,
            context_id=request.context_id,
            origin=request.origin,
            recording_policy=request.recording_policy,
            command=request.command,
            stop_request_ids=request.stop_request_ids,
            execution_instance_id=request.execution_instance_id,
            stop_execution_instance_ids=request.stop_execution_instance_ids,
        )

    def _prepare_request_locked(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> CommandRequest[SemanticCommand]:
        prepared = request
        for preparer in tuple(self._request_preparers):
            prepared = self._normalize_request(preparer(prepared))
            if prepared.request_id != request.request_id:
                raise ValueError(
                    "Request preparer must not change the request ID"
                )
        return prepared

    def _check_acknowledgement_locked(self) -> None:
        if (self._active is None or self._active_acknowledged
                or self._active_dispatched_monotonic is None
                or self._active_state in {CommandControllerState.UNKNOWN,
                                          CommandControllerState.STOP_UNCONFIRMED}):
            return
        if self._monotonic_clock() - self._active_dispatched_monotonic < self._ack_timeout_sec:
            return
        self._active_state = (CommandControllerState.STOP_UNCONFIRMED
                              if self._active_state is CommandControllerState.STOP_REQUESTED
                              else CommandControllerState.UNKNOWN)
        self._emit_locked(
            self._active, self._active_state,
            "Behavior tree did not acknowledge the dispatched "
            f"{self._active.command.command_id.value} request within "
            f"{self._ack_timeout_sec:.1f} s; execution is unknown, lane retained",
        )

    def _dispatch_consumer_ready_locked(self, *, emergency=False, resetting=False) -> bool:
        if self._dispatch_request is None:
            return False
        ready = (self._dispatch_reset_ready if resetting else
                 self._dispatch_emergency_ready if emergency else self._dispatch_ready)
        if ready is None:
            return True
        try:
            return bool(ready())
        except Exception as exception:
            self._handle_listener_error(exception)
            return False

    def _dispatch_next_locked(self) -> None:
        if self._admission_closed or self._active is not None or self._stops or self._pending_emergencies or not self._queue:
            return
        resetting = self._queue[0].command.command_id is CommandID.RESET_SOFTWARE_STOP
        if self._software_stop.latched and not resetting:
            return
        if not self._dispatch_consumer_ready_locked(resetting=resetting):
            return
        self._active = self._queue.popleft()
        self._active_state = CommandControllerState.QUEUED
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._dispatch_locked(self._active)

    def _submit_emergency(self, request) -> str:
        """Publish stop intent before touching setup state; poll does bookkeeping."""
        with self._emergency_lock:
            if request.request_id in self._emergency_request_ids:
                return request.request_id
            if request.request_id in self._known_request_ids:
                raise DuplicateCommandRequest(f"Duplicate request ID: {request.request_id}")
            if self._current_cancel_request_id is not None and self._reset_request_id is None:
                return self._current_cancel_request_id
            targets = tuple(sorted(self._unresolved_dispatch_ids))
            request = replace(request, stop_request_ids=targets,
                              stop_execution_instance_ids=tuple(self._request_execution_instances.get(key, "") for key in targets))
            self._current_cancel_request_id = request.request_id
            self._reset_request_id = None
            self._owner_stop_confirmed = False
            self._participant_stop_confirmed = self._stop_participant is None
            self._participant_stop_detail = "Waiting for acquisition abort confirmation"
            self._publish_software_stop_locked(latched=True, confirmed=False,
                detail="Software stop latched; cancellation requested, stopping is unconfirmed")
            # Persistence failure must never prevent a requested safety stop.
            self._refresh_session_work_locked()
            if self._stop_participant is not None:
                try:
                    self._stop_participant[0](request.request_id)
                except Exception as exception:
                    self._participant_stop_detail = f"Acquisition stop request is unconfirmed: {exception}"
            self._emergency_request_ids.add(request.request_id)
            self._emergency_feedback[request.request_id] = []
            sent_at = None
            error = ""
            try:
                callback = self._dispatch_request
                ready = self._dispatch_emergency_ready
                if callback is not None and (ready is None or ready()):
                    sent_at = self._monotonic_clock()
                    self._unresolved_dispatch_ids.add(request.request_id)
                    callback(request)
            except Exception as exception:
                error = f"Stop submission is unconfirmed: {exception}"
            self._pending_emergencies.append((request, sent_at, error))
        return request.request_id

    def _reconcile_emergencies_locked(self) -> None:
        while True:
            with self._emergency_lock:
                if not self._pending_emergencies:
                    return
                request, sent_at, error = self._pending_emergencies.popleft()
                queued = list(self._queue)
                self._queue.clear()
                # A stop can overtake preparation, before any submission attempt.
                if self._active is not None and self._active.request_id not in self._unresolved_dispatch_ids:
                    queued.insert(0, self._active)
                    self._active = None
                    self._active_state = None
                self._stops[request.request_id] = {
                    "request": request, "sent_at": sent_at, "acknowledged": False, "owner_confirmed": False,
                    "state": CommandControllerState.STOP_UNCONFIRMED if error else CommandControllerState.STOP_REQUESTED,
                }
                self._last_reconciled[request.request_id] = self._monotonic_clock()
                feedback = self._emergency_feedback.pop(request.request_id)
            self._emit_accepted_locked(request)
            for pending in queued:
                self._emit_locked(pending, CommandControllerState.CANCELLED,
                                  "Cancelled before dispatch by emergency stop", execution_idle=True)
            if self._active is not None:
                self._active_state = CommandControllerState.STOP_REQUESTED
                self._emit_locked(self._active, CommandControllerState.STOP_REQUESTED,
                                  "Stop requested; waiting for execution-owner confirmation")
            self._emit_locked(request, self._stops[request.request_id]["state"], error or
                              "Stop requested; waiting for execution-owner confirmation")
            for status, received_at in feedback:
                self.handle_execution_status(status, receipt_monotonic=received_at)

    def _poll_stops_locked(self):
        now = self._monotonic_clock()
        for stop in tuple(self._stops.values()):
            request = stop["request"]
            if stop["sent_at"] is None and self._dispatch_consumer_ready_locked(emergency=True):
                # Only a stop that has never reached the transport may be sent.
                stop["sent_at"] = now
                try:
                    with self._emergency_lock:
                        self._unresolved_dispatch_ids.add(request.request_id)
                        self._dispatch_request(request)
                except Exception as exception:
                    stop["state"] = CommandControllerState.STOP_UNCONFIRMED
                    self._emit_locked(request, stop["state"], f"Stop submission is unconfirmed: {exception}")
            if (not stop["acknowledged"] and stop["sent_at"] is not None and now - stop["sent_at"] >= self._ack_timeout_sec
                    and stop["state"] is not CommandControllerState.STOP_UNCONFIRMED
                    and request.request_id in self._stops):
                stop["state"] = CommandControllerState.STOP_UNCONFIRMED
                self._emit_locked(request, stop["state"], "Stop has not been confirmed; lane retained")
                if self._active is not None and self._active_state is CommandControllerState.STOP_REQUESTED:
                    self._active_state = CommandControllerState.STOP_UNCONFIRMED
                    self._emit_locked(self._active, self._active_state, "Stop has not been confirmed; lane retained")

    def _dispatch_locked(self, request) -> None:
        callback = self._dispatch_request
        if callback is None:
            raise RuntimeError("Command dispatch transport is not configured")
        try:
            with self._emergency_lock:
                if self._admission_closed or self._pending_emergencies or (self._software_stop.latched
                        and request.command.command_id is not CommandID.RESET_SOFTWARE_STOP):
                    return
                # Durable ownership precedes every possibly physical dispatch.
                self._set_session_work_locked(True)
                self._active_dispatched_monotonic = self._monotonic_clock()
                self._last_reconciled[request.request_id] = self._active_dispatched_monotonic
                self._unresolved_dispatch_ids.add(request.request_id)
                self._active_state = CommandControllerState.DISPATCHED
                callback(request)
        except Exception as exception:
            if self._active is request:
                self._active_state = CommandControllerState.UNKNOWN
                self._emit_locked(request, self._active_state,
                                  f"Command submission is uncertain: {exception}; lane retained")
            return
        if self._active is request and self._active_state is CommandControllerState.DISPATCHED:
            self._emit_locked(request, self._active_state,
                              f"Dispatched {request.command.command_id.value} to behavior tree; waiting for BT receipt")

    def _queued_detail_locked(self, request) -> str:
        if self._active_state in {CommandControllerState.UNKNOWN, CommandControllerState.STOP_UNCONFIRMED}:
            return "Queued; waiting for unresolved execution ownership to be reconciled"
        if self._active is not None:
            return (
                "Queued behind active request "
                f"{self._active.request_id}"
            )
        if self._stops or self._pending_emergencies:
            return "Queued; waiting for confirmed stopping"
        if not self._dispatch_consumer_ready_locked():
            return "Queued; waiting for behavior-tree command consumer"
        return "Queued for behavior-tree dispatch"

    def _emit_accepted_locked(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> None:
        for listener in tuple(self._accepted_listeners):
            self._call_listener(listener, request)

    def _emit_locked(
        self,
        request: CommandRequest[SemanticCommand],
        state: CommandControllerState,
        detail: str = "",
        retry_safe: bool = False,
        continuation_safe: bool = False,
        retry_command: SemanticCommand | None = None,
        execution_idle: bool = False,
    ) -> None:
        if state is CommandControllerState.QUEUED:
            self._last_queue_status[request.request_id] = self._monotonic_clock()
        else:
            self._last_queue_status.pop(request.request_id, None)
        self._public_status_sequence += 1
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
            buffered_command_count=len(self._queue),
            retry_safe=retry_safe,
            continuation_safe=continuation_safe,
            retry_command=retry_command,
            execution_idle=execution_idle,
            execution_instance_id=self._request_execution_instances.get(request.request_id, ""),
            status_sequence=self._public_status_sequence,
        )
        for listener in tuple(self._listeners):
            self._call_listener(listener, status)

    def _call_listener(self, listener, value) -> None:
        try:
            listener(value)
        except Exception as exception:
            self._handle_listener_error(exception)

    def _handle_listener_error(self, exception: Exception) -> None:
        if self._listener_error_handler is not None:
            self._listener_error_handler(exception)

    @staticmethod
    def _is_emergency(
        request: CommandRequest[SemanticCommand],
    ) -> bool:
        return request.command.command_id is CommandID.EMERGENCY_CANCEL


__all__ = [
    "CommandController",
    "CommandControllerState",
    "CommandControllerStatus",
    "CommandExecutionUnresolved",
    "CommandExecutionStatus",
    "DuplicateCommandRequest",
    "UnknownCommandRequest",
    "SoftwareStopStatus",
    "SoftwareStopLatched",
]
