"""Serialize semantic commands independently of ROS transport."""

from collections import deque
from dataclasses import dataclass
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


class DuplicateCommandRequest(ValueError):
    """Raised when a request identity has already been accepted."""


class UnknownCommandRequest(LookupError):
    """Raised when cancellation targets an unknown request."""


StatusListener = Callable[[CommandControllerStatus], None]
AcceptedListener = Callable[[CommandRequest[SemanticCommand]], None]
DispatchCallback = Callable[[CommandRequest[SemanticCommand]], None]
DispatchReady = Callable[[], bool]
ListenerErrorHandler = Callable[[Exception], None]


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
        self._queue: Deque[CommandRequest[SemanticCommand]] = deque()
        self._active: Optional[CommandRequest[SemanticCommand]] = None
        self._listeners: List[StatusListener] = []
        self._accepted_listeners: List[AcceptedListener] = []
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
            return self._active.request_id if self._active else ""

    @property
    def queued_request_ids(self) -> Tuple[str, ...]:
        with self._lock:
            return tuple(request.request_id for request in self._queue)

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

    def clear_dispatch(self) -> None:
        """Detach the execution transport without changing queued requests."""
        with self._lock:
            self._dispatch_request = None
            self._dispatch_ready = None

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

    def submit(self, request: CommandRequest[SemanticCommand]) -> str:
        normalized = self._normalize_request(request)
        with self._lock:
            if normalized.request_id in self._known_request_ids:
                raise DuplicateCommandRequest(
                    f"Duplicate request ID: {normalized.request_id}"
                )
            self._known_request_ids.add(normalized.request_id)
            self._emit_accepted_locked(normalized)
            if self._is_emergency(normalized):
                self._dispatch_emergency_locked(normalized)
            else:
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

    def handle_execution_status(
        self,
        status: CommandExecutionStatus,
    ) -> bool:
        if not isinstance(status, CommandExecutionStatus):
            raise TypeError("Expected a CommandExecutionStatus")
        if status.state not in {
            CommandControllerState.RUNNING,
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }:
            return False
        with self._lock:
            if (
                self._active is None
                or status.request_id != self._active.request_id
            ):
                return False
            request = self._active
            self._active_acknowledged = True
            if status.state is CommandControllerState.RUNNING:
                self._emit_locked(
                    request,
                    CommandControllerState.RUNNING,
                    status.detail,
                )
                return True
            if status.state is CommandControllerState.SUCCEEDED:
                if status.buffered_command_count > 0:
                    self._emit_locked(
                        request,
                        CommandControllerState.RUNNING,
                        status.detail,
                    )
                    return True
                state = CommandControllerState.SUCCEEDED
            else:
                state = status.state
            self._active = None
            self._active_acknowledged = False
            self._active_dispatched_monotonic = None
            self._emit_locked(
                request,
                state,
                status.detail,
            )
            self._dispatch_next_locked()
            return True

    def poll(self) -> None:
        """Advance timeout handling and dispatch queued work when possible."""
        with self._lock:
            self._retry_dispatch_locked()
            self._dispatch_next_locked()

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
        )

    def _retry_dispatch_locked(self) -> None:
        if (
            self._active is None
            or self._active_acknowledged
            or self._active_dispatched_monotonic is None
        ):
            return
        if (
            self._monotonic_clock()
            - self._active_dispatched_monotonic
            < self._ack_timeout_sec
        ):
            return
        request = self._active
        self._active = None
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        detail = (
            "Behavior tree did not acknowledge the dispatched "
            f"{request.command.command_id.value} request within "
            f"{self._ack_timeout_sec:.1f} s"
        )
        self._emit_locked(
            request,
            CommandControllerState.FAILED,
            detail,
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
        if self._active is not None or not self._queue:
            return
        if not self._dispatch_consumer_ready_locked():
            return
        self._active = self._queue.popleft()
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._dispatch_locked(self._active)

    def _dispatch_emergency_locked(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> None:
        interrupted = []
        if self._active is not None:
            interrupted.append(self._active)
        interrupted.extend(self._queue)
        self._active = None
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._queue.clear()
        for pending in interrupted:
            self._emit_locked(
                pending,
                CommandControllerState.CANCELLED,
                "Interrupted by emergency stop",
            )
        if self._dispatch_consumer_ready_locked():
            self._active = request
            self._dispatch_locked(request)
        else:
            self._queue.appendleft(request)
            self._emit_locked(
                request,
                CommandControllerState.QUEUED,
                "Waiting for behavior-tree command consumer",
            )

    def _dispatch_locked(
        self,
        request: CommandRequest[SemanticCommand],
    ) -> None:
        callback = self._dispatch_request
        if callback is None:
            raise RuntimeError("Command dispatch transport is not configured")
        self._active_dispatched_monotonic = self._monotonic_clock()
        try:
            callback(request)
        except Exception as exception:
            if self._active is request:
                self._active = None
                self._active_acknowledged = False
                self._active_dispatched_monotonic = None
                self._emit_locked(
                    request,
                    CommandControllerState.FAILED,
                    f"Command dispatch failed: {exception}",
                )
                self._dispatch_next_locked()
            return
        if self._active is not request:
            return
        command_id = request.command.command_id.value
        self._emit_locked(
            request,
            CommandControllerState.DISPATCHED,
            f"Dispatched {command_id} to behavior tree; "
            "waiting for BT receipt",
        )

    def _queued_detail_locked(self, request) -> str:
        if self._active is not None:
            return (
                "Queued behind active request "
                f"{self._active.request_id}"
            )
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
    ) -> None:
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
            buffered_command_count=len(self._queue),
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
