"""Serialize semantic commands before behavior-tree execution."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from threading import RLock
from typing import Callable, Deque, List, Optional, Tuple

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    CommandStatus,
    ComplexCommand,
)

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_to_message,
)
from fault_detector_spot.shared.ros.qos_profiles import (
    COMMAND_REQUEST_QOS,
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

    request: CommandRequest[ComplexCommand]
    state: CommandControllerState
    detail: str = ""
    buffered_command_count: int = 0

    @property
    def request_id(self) -> str:
        """Return the correlated request identity."""
        return self.request.request_id


class DuplicateCommandRequest(ValueError):
    """Raised when a request identity has already been accepted."""


class UnknownCommandRequest(LookupError):
    """Raised when cancellation targets an unknown request."""


StatusListener = Callable[[CommandControllerStatus], None]


class CommandController:
    """Own the single semantic command queue and result correlation."""

    def __init__(
        self,
        node,
        dispatch_topic: str = "fault_detector/commands/request",
        accepted_topic: str = "fault_detector/commands/accepted",
        status_topic: str = "fault_detector/command_status",
    ):
        self.node = node
        self._lock = RLock()
        self._queue: Deque[CommandRequest[ComplexCommand]] = deque()
        self._active: Optional[CommandRequest[ComplexCommand]] = None
        self._listeners: List[StatusListener] = []
        self._known_request_ids = set()
        self._dispatch_publisher = node.create_publisher(
            CommandRequestMessage,
            dispatch_topic,
            COMMAND_REQUEST_QOS,
        )
        self._accepted_publisher = node.create_publisher(
            CommandRequestMessage,
            accepted_topic,
            COMMAND_REQUEST_QOS,
        )
        self._status_subscription = node.create_subscription(
            CommandStatus,
            status_topic,
            self.handle_command_status,
            10,
        )

    @property
    def active_request_id(self) -> str:
        """Return the active request identity, or an empty string."""
        with self._lock:
            return self._active.request_id if self._active else ""

    @property
    def queued_request_ids(self) -> Tuple[str, ...]:
        """Return queued request identities in dispatch order."""
        with self._lock:
            return tuple(request.request_id for request in self._queue)

    def add_status_listener(self, listener: StatusListener) -> None:
        """Register a semantic command status listener."""
        if not callable(listener):
            raise TypeError("Status listener must be callable")
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_status_listener(self, listener: StatusListener) -> None:
        """Remove a previously registered status listener."""
        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def submit(
        self,
        request: CommandRequest[ComplexCommand],
    ) -> str:
        """Accept and queue one validated semantic command."""
        normalized = self._normalize_request(request)
        with self._lock:
            if normalized.request_id in self._known_request_ids:
                raise DuplicateCommandRequest(
                    f"Duplicate request ID: {normalized.request_id}"
                )
            self._known_request_ids.add(normalized.request_id)
            self._accepted_publisher.publish(
                command_request_to_message(normalized)
            )
            if self._is_emergency(normalized):
                self._dispatch_emergency_locked(normalized)
            else:
                self._queue.append(normalized)
                self._emit_locked(
                    normalized,
                    CommandControllerState.QUEUED,
                )
                self._dispatch_next_locked()
        return normalized.request_id

    def cancel(self, request_id: str) -> str:
        """Cancel a queued request or globally stop an active request."""
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

    def cancel_all(self, client_id: str = "command_controller") -> str:
        """Clear queued work and dispatch the behavior-tree emergency stop."""
        command = ComplexCommand()
        command.command.command_id = CommandID.EMERGENCY_CANCEL.value
        request = CommandRequest.create(
            command=command,
            client_id=client_id,
            origin=CommandOrigin.SYSTEM,
            recording_policy=RecordingPolicy.EXCLUDE,
        )
        return self.submit(request)

    def handle_command_status(self, message: CommandStatus) -> bool:
        """Consume one behavior-tree status for the active request."""
        with self._lock:
            if (
                self._active is None
                or message.request_id != self._active.request_id
            ):
                return False
            request = self._active
            if message.state == CommandStatus.STATE_RUNNING:
                self._emit_locked(
                    request,
                    CommandControllerState.RUNNING,
                    message.detail,
                    message.buffered_command_count,
                )
                return True
            if message.state == CommandStatus.STATE_SUCCEEDED:
                if message.buffered_command_count > 0:
                    self._emit_locked(
                        request,
                        CommandControllerState.RUNNING,
                        message.detail,
                        message.buffered_command_count,
                    )
                    return True
                state = CommandControllerState.SUCCEEDED
            elif message.state == CommandStatus.STATE_FAILED:
                state = CommandControllerState.FAILED
            elif message.state == CommandStatus.STATE_CANCELLED:
                state = CommandControllerState.CANCELLED
            else:
                return False
            self._active = None
            self._emit_locked(
                request,
                state,
                message.detail,
                message.buffered_command_count,
            )
            self._dispatch_next_locked()
            return True

    def _normalize_request(
        self,
        request: CommandRequest[ComplexCommand],
    ) -> CommandRequest[ComplexCommand]:
        message = command_request_to_message(request)
        return CommandRequest(
            request_id=message.request_id,
            client_id=message.client_id,
            context_id=message.context_id,
            origin=message.origin,
            recording_policy=message.recording_policy,
            command=message.command,
        )

    def _dispatch_next_locked(self) -> None:
        if self._active is not None or not self._queue:
            return
        self._active = self._queue.popleft()
        self._publish_dispatch_locked(self._active)

    def _dispatch_emergency_locked(
        self,
        request: CommandRequest[ComplexCommand],
    ) -> None:
        interrupted = []
        if self._active is not None:
            interrupted.append(self._active)
        interrupted.extend(self._queue)
        self._active = None
        self._queue.clear()
        for pending in interrupted:
            self._emit_locked(
                pending,
                CommandControllerState.CANCELLED,
                "Interrupted by emergency stop",
            )
        self._active = request
        self._publish_dispatch_locked(request)

    def _publish_dispatch_locked(
        self,
        request: CommandRequest[ComplexCommand],
    ) -> None:
        message = command_request_to_message(request)
        message.command.command.header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        self._dispatch_publisher.publish(message)
        self._emit_locked(
            request,
            CommandControllerState.DISPATCHED,
        )

    def _emit_locked(
        self,
        request: CommandRequest[ComplexCommand],
        state: CommandControllerState,
        detail: str = "",
        buffered_command_count: int = 0,
    ) -> None:
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
            buffered_command_count=buffered_command_count,
        )
        for listener in tuple(self._listeners):
            try:
                listener(status)
            except Exception as exception:
                self.node.get_logger().error(
                    f"Command status listener failed: {exception}"
                )

    @staticmethod
    def _is_emergency(
        request: CommandRequest[ComplexCommand],
    ) -> bool:
        return (
            request.command.command.command_id
            == CommandID.EMERGENCY_CANCEL.value
        )
