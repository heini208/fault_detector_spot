"""Serialize semantic commands before behavior-tree execution."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from threading import RLock
import time
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
    command_request_from_message,
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
        submission_topic: str = "fault_detector/_internal/commands/submit",
        dispatch_topic: str = "fault_detector/_internal/commands/request",
        accepted_topic: str = "fault_detector/_internal/commands/accepted",
        controller_status_topic: str = (
            "fault_detector/_internal/commands/status"
        ),
        status_topic: str = "fault_detector/_internal/command_status",
    ):
        self.node = node
        self._lock = RLock()
        self._queue: Deque[CommandRequest[ComplexCommand]] = deque()
        self._active: Optional[CommandRequest[ComplexCommand]] = None
        self._listeners: List[StatusListener] = []
        self._known_request_ids = set()
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
        self._ack_timeout_sec = 3.0
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
        self._controller_status_publisher = node.create_publisher(
            CommandStatus,
            controller_status_topic,
            10,
        )
        self._submission_subscription = node.create_subscription(
            CommandRequestMessage,
            submission_topic,
            self.submit_message,
            COMMAND_REQUEST_QOS,
        )
        self._status_subscription = node.create_subscription(
            CommandStatus,
            status_topic,
            self.handle_command_status,
            10,
        )
        create_timer = getattr(node, "create_timer", None)
        self._dispatch_retry_timer = (
            create_timer(0.1, self._retry_dispatch)
            if callable(create_timer)
            else None
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
                    self._queued_detail_locked(normalized),
                )
                self._dispatch_next_locked()
        return normalized.request_id

    def submit_message(self, message: CommandRequestMessage) -> bool:
        """Validate and submit one ROS-facing semantic request."""
        try:
            request = command_request_from_message(message)
            self.submit(request)
        except (TypeError, ValueError) as exception:
            self._reject_message(message, str(exception))
            return False
        return True

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
            self._active_acknowledged = True
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
            self._active_acknowledged = False
            self._active_dispatched_monotonic = None
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

    def _retry_dispatch(self) -> None:
        with self._lock:
            if (
                self._active is not None
                and not self._active_acknowledged
                and self._active_dispatched_monotonic is not None
                and (
                    time.monotonic() - self._active_dispatched_monotonic
                    >= self._ack_timeout_sec
                )
            ):
                request = self._active
                self._active = None
                self._active_acknowledged = False
                self._active_dispatched_monotonic = None
                detail = (
                    "Behavior tree did not acknowledge the dispatched "
                    f"{request.command.command.command_id} request within "
                    f"{self._ack_timeout_sec:.1f} s"
                )
                self.node.get_logger().error(
                    f"{detail} [{request.request_id}]"
                )
                self._emit_locked(
                    request,
                    CommandControllerState.FAILED,
                    detail,
                )
            self._dispatch_next_locked()

    def _dispatch_consumer_ready_locked(self) -> bool:
        get_count = getattr(
            self._dispatch_publisher,
            "get_subscription_count",
            None,
        )
        if not callable(get_count):
            return True
        try:
            return int(get_count()) > 0
        except Exception:
            return False

    def _dispatch_next_locked(self) -> None:
        if self._active is not None or not self._queue:
            return
        if not self._dispatch_consumer_ready_locked():
            return
        self._active = self._queue.popleft()
        self._active_acknowledged = False
        self._active_dispatched_monotonic = None
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
            self._publish_dispatch_locked(request)
        else:
            self._queue.appendleft(request)
            self._emit_locked(
                request,
                CommandControllerState.QUEUED,
                "Waiting for behavior-tree command consumer",
            )

    def _publish_dispatch_locked(
        self,
        request: CommandRequest[ComplexCommand],
    ) -> None:
        message = command_request_to_message(request)
        message.command.command.header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        self._dispatch_publisher.publish(message)
        self._active_dispatched_monotonic = time.monotonic()
        command_id = request.command.command.command_id
        detail = (
            f"Published {command_id} to behavior tree; "
            "waiting for BT receipt"
        )
        self._log_info(
            f"{detail} [{request.request_id}]"
        )
        self._emit_locked(
            request,
            CommandControllerState.DISPATCHED,
            detail,
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

    def _log_info(self, message: str) -> None:
        logger_factory = getattr(self.node, "get_logger", None)
        if not callable(logger_factory):
            return
        logger = logger_factory()
        info = getattr(logger, "info", None)
        if callable(info):
            info(message)

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
        self._publish_controller_status(status)
        for listener in tuple(self._listeners):
            try:
                listener(status)
            except Exception as exception:
                self.node.get_logger().error(
                    f"Command status listener failed: {exception}"
                )

    def _reject_message(self, message, detail):
        command = getattr(message, "command", None)
        basic = getattr(command, "command", None)
        self._publish_rejection(
            getattr(message, "request_id", ""),
            getattr(basic, "command_id", ""),
            detail,
        )

    def _publish_rejection(self, request_id, command_id, detail):
        self.node.get_logger().error(f"Rejected command request: {detail}")
        if not request_id:
            return
        message = CommandStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = request_id
        message.command_id = command_id
        message.state = CommandStatus.STATE_FAILED
        message.detail = detail
        message.buffered_command_count = 0
        self._controller_status_publisher.publish(message)

    def _publish_controller_status(self, status):
        message = CommandStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = status.request_id
        message.command_id = (
            status.request.command.command.command_id
        )
        message.state = self._controller_status_state(status.state)
        message.detail = status.detail or status.state.value
        message.buffered_command_count = status.buffered_command_count
        self._controller_status_publisher.publish(message)

    @staticmethod
    def _controller_status_state(state):
        if state in (
            CommandControllerState.QUEUED,
            CommandControllerState.DISPATCHED,
            CommandControllerState.RUNNING,
        ):
            return CommandStatus.STATE_RUNNING
        if state is CommandControllerState.SUCCEEDED:
            return CommandStatus.STATE_SUCCEEDED
        if state is CommandControllerState.FAILED:
            return CommandStatus.STATE_FAILED
        if state is CommandControllerState.CANCELLED:
            return CommandStatus.STATE_CANCELLED
        raise ValueError(f"Unsupported controller state: {state}")

    @staticmethod
    def _is_emergency(
        request: CommandRequest[ComplexCommand],
    ) -> bool:
        return (
            request.command.command.command_id
            == CommandID.EMERGENCY_CANCEL.value
        )
