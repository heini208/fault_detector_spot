"""Retain the BT owner's receipts and terminal evidence for read-only queries."""

from copy import deepcopy
from threading import RLock

from fault_detector_msgs.msg import CommandStatus


class CommandStatusLedger:
    """Fence stopped identities and replay evidence without replaying commands."""

    def __init__(self):
        self._lock = RLock()
        self._statuses = {}
        self._fenced = set()
        self._stopping = False
        self._active_stop_id = None

    def receive(self, request, *, emergency=False):
        """Return whether this is a new admissible request and its receipt."""
        with self._lock:
            if request.request_id in self._statuses:
                return False, deepcopy(self._statuses[request.request_id])
            message = CommandStatus()
            message.request_id = request.request_id
            message.command_id = request.command.command_id.value
            message.state = CommandStatus.STATE_RUNNING
            message.detail = "BT received command request"
            if self._stopping and not emergency:
                message.state = CommandStatus.STATE_FAILED
                message.detail = "Stop is pending; this request was not executed"
                message.execution_idle = True
            self._statuses[request.request_id] = deepcopy(message)
            return not (self._stopping and not emergency), message

    def request_stop(self, request):
        """Install fences before acknowledging a stop, including unseen targets."""
        with self._lock:
            self._stopping = True
            self._active_stop_id = request.request_id
            targets = set(request.stop_request_ids)
            targets.add(request.request_id)
            targets.update(
                key for key, value in self._statuses.items()
                if not value.execution_idle
            )
            self._fenced.update(targets)
            for request_id in targets:
                previous = self._statuses.get(request_id)
                message = deepcopy(previous) if previous is not None else CommandStatus()
                message.request_id = request_id
                message.state = CommandStatus.STATE_STOP_REQUESTED
                message.detail = "BT stop requested; execution is not yet confirmed idle"
                message.execution_idle = False
                message.buffered_command_count = 0
                message.retry_safe = False
                message.continuation_safe = False
                self._statuses[request_id] = message

    def record(self, message):
        if not message.request_id:
            return deepcopy(message)
        with self._lock:
            previous = self._statuses.get(message.request_id)
            if previous is not None and previous.execution_idle:
                return deepcopy(previous)
            if (message.request_id in self._fenced
                    and message.state not in {
                        CommandStatus.STATE_STOP_REQUESTED,
                        CommandStatus.STATE_STOP_UNCONFIRMED,
                        CommandStatus.STATE_STOP_CONFIRMED,
                    }):
                return deepcopy(previous)
            self._statuses[message.request_id] = deepcopy(message)
            return deepcopy(message)

    def confirm_stop(self, message):
        """Confirm every fenced BT identity only after global owner quiescence."""
        if message.state != CommandStatus.STATE_STOP_CONFIRMED or not message.execution_idle:
            raise ValueError("Stop completion requires explicit idle evidence")
        with self._lock:
            if message.request_id != self._active_stop_id:
                return []
            confirmed = []
            for request_id in sorted(self._fenced):
                status = deepcopy(self._statuses[request_id])
                status.header = deepcopy(message.header)
                status.state = CommandStatus.STATE_STOP_CONFIRMED
                status.detail = message.detail
                status.execution_idle = True
                status.buffered_command_count = 0
                status.retry_safe = False
                status.continuation_safe = False
                self._statuses[request_id] = status
                confirmed.append(deepcopy(status))
            self._stopping = False
            self._active_stop_id = None
            return confirmed

    def contains(self, request_id):
        with self._lock:
            return request_id in self._statuses

    def query(self, request_id):
        with self._lock:
            if request_id in self._statuses:
                return deepcopy(self._statuses[request_id])
        message = CommandStatus()
        message.request_id = request_id
        message.state = CommandStatus.STATE_UNKNOWN
        message.detail = "BT owner has no retained evidence for this request"
        message.execution_idle = False
        return message
