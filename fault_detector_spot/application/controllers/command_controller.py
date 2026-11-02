"""Serialize semantic commands independently of ROS transport."""

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
from threading import RLock
import time
from typing import Callable, Deque, List, Optional, Tuple

from fault_detector_spot.application.commanding.command_ids import CommandID
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


class DuplicateCommandRequest(ValueError):
    """Raised when a request identity has already been accepted."""


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
    ):
        if dispatch_request is not None and not callable(dispatch_request):
            raise TypeError("Dispatch callback must be callable")
        if dispatch_ready is not None and not callable(dispatch_ready):
            raise TypeError("Dispatch readiness callback must be callable")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        if ack_timeout_sec <= 0.0:
            raise ValueError("Acknowledgement timeout must be positive")
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
        self._stops = {}
        self._active_state = None
        self._reconcile_request = None
        self._last_reconciled = {}
        self._queue: Deque[CommandRequest[SemanticCommand]] = deque()
        self._active: Optional[CommandRequest[SemanticCommand]] = None
        self._listeners: List[StatusListener] = []
        self._accepted_listeners: List[AcceptedListener] = []
        self._request_preparers: List[RequestPreparer] = []
        self._known_request_ids = set()
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._ack_timeout_sec = float(ack_timeout_sec)
        self._monotonic_clock = monotonic_clock
        self._dispatch_request = dispatch_request
        self._dispatch_ready = dispatch_ready
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

    def configure_dispatch(
        self,
        dispatch_request: DispatchCallback,
        dispatch_ready: Optional[DispatchReady] = None,
    ) -> None:
        """Attach the execution transport used for future dispatches."""
        if not callable(dispatch_request):
            raise TypeError("Dispatch callback must be callable")
        if dispatch_ready is not None and not callable(dispatch_ready):
            raise TypeError("Dispatch readiness callback must be callable")
        with self._lock:
            self._dispatch_request = dispatch_request
            self._dispatch_ready = dispatch_ready
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
        if not callable(action):
            raise TypeError("Idle-lane action must be callable")
        if not isinstance(busy_message, str) or not busy_message.strip():
            raise ValueError("Busy-lane message must not be empty")
        with self._lock:
            if self._active is not None or self._queue or self._stops or self._pending_emergencies:
                raise RuntimeError(busy_message.strip())
            return action()

    def submit(self, request: CommandRequest[SemanticCommand]) -> str:
        normalized = self._normalize_request(request)
        if self._is_emergency(normalized):
            return self._submit_emergency(normalized)
        with self._lock:
            normalized = self._prepare_request_locked(normalized)
            with self._emergency_lock:
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

    def handle_execution_status(self, status: CommandExecutionStatus) -> bool:
        """Apply correlated owner evidence, never infer stopping from silence."""
        if not isinstance(status, CommandExecutionStatus):
            raise TypeError("Expected a CommandExecutionStatus")
        if status.state not in {
            CommandControllerState.RUNNING, CommandControllerState.UNKNOWN,
            CommandControllerState.SUCCEEDED, CommandControllerState.FAILED,
            CommandControllerState.CANCELLED, CommandControllerState.STOP_REQUESTED,
            CommandControllerState.STOP_CONFIRMED, CommandControllerState.STOP_UNCONFIRMED,
        }:
            return False
        with self._emergency_lock:
            feedback = self._emergency_feedback.get(status.request_id)
            if feedback is not None:
                feedback.append(status)
                return True
        with self._lock:
            self._reconcile_emergencies_locked()
            stop = self._stops.get(status.request_id)
            if stop is not None:
                confirmed = (
                    status.state is CommandControllerState.STOP_CONFIRMED
                    and status.execution_idle and status.buffered_command_count == 0
                )
                if confirmed:
                    del self._stops[status.request_id]
                    self._forget_dispatch_locked(status.request_id)
                    self._emit_locked(stop["request"], CommandControllerState.STOP_CONFIRMED,
                                      status.detail, execution_idle=True)
                    self._dispatch_next_locked()
                else:
                    state = (CommandControllerState.STOP_REQUESTED
                             if status.state in {CommandControllerState.RUNNING,
                                                 CommandControllerState.STOP_REQUESTED}
                             and stop["state"] is not CommandControllerState.STOP_UNCONFIRMED
                             else CommandControllerState.STOP_UNCONFIRMED)
                    stop["state"] = state
                    self._emit_locked(stop["request"], state, status.detail or
                                      "Stop is not yet confirmed by the execution owner")
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
            self._active_state = state
            detail = status.detail
            if terminal and state in {CommandControllerState.UNKNOWN, CommandControllerState.STOP_UNCONFIRMED}:
                detail += "; execution owner has not confirmed this request stopped"
            self._emit_locked(request, state, detail)
            return True

    def poll(self) -> None:
        """Reconcile retained requests without ever repeating uncertain motion."""
        with self._lock:
            self._reconcile_emergencies_locked()
            self._check_acknowledgement_locked()
            self._poll_stops_locked()
            self._query_owners_locked()
            self._dispatch_next_locked()

    def _forget_dispatch_locked(self, request_id):
        with self._emergency_lock:
            self._unresolved_dispatch_ids.discard(request_id)
        self._last_reconciled.pop(request_id, None)

    def _query_owners_locked(self):
        if self._reconcile_request is None:
            return
        now = self._monotonic_clock()
        request_ids = list(self._stops)
        if self._active is not None:
            request_ids.append(self._active.request_id)
        for request_id in request_ids:
            previous = self._last_reconciled.get(request_id, now)
            if now - previous < self._ack_timeout_sec:
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

    def _dispatch_consumer_ready_locked(self) -> bool:
        if self._dispatch_request is None:
            return False
        if self._dispatch_ready is None:
            return True
        try:
            return bool(self._dispatch_ready())
        except Exception as exception:
            self._handle_listener_error(exception)
            return False

    def _dispatch_next_locked(self) -> None:
        if self._active is not None or self._stops or self._pending_emergencies or not self._queue:
            return
        if not self._dispatch_consumer_ready_locked():
            return
        self._active = self._queue.popleft()
        self._active_state = CommandControllerState.QUEUED
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._dispatch_locked(self._active)

    def _submit_emergency(self, request) -> str:
        """Publish stop intent before touching setup state; poll does bookkeeping."""
        with self._emergency_lock:
            if request.request_id in self._known_request_ids or request.request_id in self._emergency_request_ids:
                raise DuplicateCommandRequest(f"Duplicate request ID: {request.request_id}")
            request = replace(request, stop_request_ids=tuple(sorted(self._unresolved_dispatch_ids)))
            self._emergency_request_ids.add(request.request_id)
            self._emergency_feedback[request.request_id] = []
            sent_at = None
            error = ""
            try:
                callback = self._dispatch_request
                ready = self._dispatch_ready
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
                    "request": request, "sent_at": sent_at,
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
            for status in feedback:
                self.handle_execution_status(status)

    def _poll_stops_locked(self):
        now = self._monotonic_clock()
        for stop in tuple(self._stops.values()):
            request = stop["request"]
            if stop["sent_at"] is None and self._dispatch_consumer_ready_locked():
                # Only a stop that has never reached the transport may be sent.
                stop["sent_at"] = now
                try:
                    with self._emergency_lock:
                        self._unresolved_dispatch_ids.add(request.request_id)
                        self._dispatch_request(request)
                except Exception as exception:
                    stop["state"] = CommandControllerState.STOP_UNCONFIRMED
                    self._emit_locked(request, stop["state"], f"Stop submission is unconfirmed: {exception}")
            if (stop["sent_at"] is not None and now - stop["sent_at"] >= self._ack_timeout_sec
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
                if self._pending_emergencies:
                    return
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
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
            buffered_command_count=len(self._queue),
            retry_safe=retry_safe,
            continuation_safe=continuation_safe,
            retry_command=retry_command,
            execution_idle=execution_idle,
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
    "CommandExecutionStatus",
    "DuplicateCommandRequest",
    "UnknownCommandRequest",
]
