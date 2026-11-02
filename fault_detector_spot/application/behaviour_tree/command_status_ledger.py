"""Retain the BT owner's receipts and terminal evidence for read-only queries."""

from copy import deepcopy
from threading import RLock
from uuid import uuid4

from fault_detector_msgs.msg import CommandStatus
from fault_detector_spot.application.commanding.command_ids import CommandID


class CommandStatusLedger:
    """Fence stopped identities and replay evidence without replaying commands."""

    def __init__(self):
        self._lock = RLock()
        self._statuses = {}
        self._fenced = set()
        self._stopping = False
        self._active_stop_id = None
        self.execution_instance_id = str(uuid4())
        self._status_sequence = 0
        self._activated = False
        self._foreign_targets = set()
        self._restored_shutdown_targets = set()
        self._closing = False
        self._shutdown_result = None

    def activate(self):
        with self._lock:
            if not self._closing:
                self._activated = True

    def restore_shutdown_unconfirmed(self, resources):
        """Fence prior process work that new execution owners cannot confirm."""
        with self._lock:
            for resource, detail in resources.items():
                request_id = f"shutdown:{resource}"
                self._foreign_targets.add(request_id)
                self._restored_shutdown_targets.add(request_id)
                self._fenced.add(request_id)
                status = self.unknown_status(request_id, detail)
                status.state = CommandStatus.STATE_STOP_UNCONFIRMED
                self._statuses[request_id] = status
            if resources:
                self._stopping = True

    def close_admission(self):
        """Fence ingress while callback resources remain available to stop."""
        with self._lock:
            if self._closing:
                return
            self._closing = True
            self._activated = False
            self._stopping = True
            self._active_stop_id = None
            for request_id, status in self._statuses.items():
                if not status.execution_idle:
                    self._fenced.add(request_id)
                    if request_id not in self._foreign_targets:
                        status.state = CommandStatus.STATE_STOP_REQUESTED
                        status.detail = "Shutdown requested; waiting for terminal actions and physical stop evidence"
                        status.buffered_command_count = 0

    def report_shutdown_result(self, unresolved):
        """Apply only the shutdown coordinator's complete owner evidence."""
        with self._lock:
            self.close_admission()
            if self._shutdown_result is None:
                self._shutdown_result = dict(unresolved)
            unresolved = self._shutdown_result
            confirmed = not unresolved and not self._foreign_targets
            detail = ("Shutdown stop confirmed by all execution owners" if confirmed else
                      "Shutdown stop-unconfirmed; reconciliation is required: " +
                      "; ".join(f"{name}: {reason}" for name, reason in unresolved.items()))
            for request_id in self._fenced:
                status = self._statuses[request_id]
                if status.execution_idle or request_id in self._foreign_targets:
                    continue
                status.state = (CommandStatus.STATE_STOP_CONFIRMED if confirmed else
                                CommandStatus.STATE_STOP_UNCONFIRMED)
                status.detail = detail
                status.execution_idle = confirmed
                status.buffered_command_count = 0
                status.retry_safe = False
                status.continuation_safe = False
            return [deepcopy(self._statuses[key]) for key in sorted(self._fenced)]

    def join_shutdown_stop(self, request):
        """Join existing shutdown proof without submitting another stop workflow."""
        with self._lock:
            self.close_admission()
            if request.request_id in self._statuses:
                return [deepcopy(self._statuses[request.request_id])]
            self.request_stop(request)
            self._active_stop_id = None
            self._statuses[request.request_id].command_id = request.command.command_id.value
            if self._shutdown_result is not None:
                return self.report_shutdown_result(self._shutdown_result)
            return [deepcopy(self._statuses[key]) for key in sorted(self._fenced)]

    def shutdown_ownership_status(self):
        """New local idle owners cannot certify a foreign request's termination."""
        with self._lock:
            # Restored persistent resources have their own coordinator entries.
            # Repeating those here would recursively duplicate marker details.
            foreign = sorted(self._foreign_targets - self._restored_shutdown_targets)
            return not foreign, "; ".join(
                f"{key}: {self._statuses[key].detail}" for key in foreign
            ) or "All retained request ownership belongs to this execution instance"

    def outgoing_status(self, message):
        """Sequence each actual publication or fresh query reply, not tree ticks."""
        with self._lock:
            result = deepcopy(message)
            self._status_sequence += 1
            result.execution_instance_id = self.execution_instance_id
            result.status_sequence = self._status_sequence
            result.software_stop_latched = self._stopping
            result.software_stop_confirmed = (
                not self._foreign_targets and bool(self._fenced)
                and all(status.execution_idle for status in self._statuses.values())
            )
            return result

    def unknown_status(self, request_id, detail):
        message = CommandStatus()
        message.request_id = request_id
        message.execution_instance_id = self.execution_instance_id
        message.state = CommandStatus.STATE_UNKNOWN
        message.detail = detail
        message.execution_idle = False
        return message

    def receive(self, request, *, emergency=False):
        """Return whether this is a new admissible request and its receipt."""
        with self._lock:
            if request.execution_instance_id != self.execution_instance_id:
                return False, self.unknown_status(
                    request.request_id, "Command targets another execution instance",
                )
            if request.request_id in self._statuses:
                return False, deepcopy(self._statuses[request.request_id])
            message = CommandStatus()
            message.request_id = request.request_id
            message.execution_instance_id = self.execution_instance_id
            message.command_id = request.command.command_id.value
            message.state = CommandStatus.STATE_RUNNING
            message.detail = "BT received command request"
            reset = request.command.command_id is CommandID.RESET_SOFTWARE_STOP
            reset_ready = (self._stopping and not self._foreign_targets
                           and all(item.execution_idle for item in self._statuses.values()))
            blocked = (self._closing or (self._stopping and not emergency and not reset)
                       or (reset and not reset_ready))
            if blocked:
                message.state = CommandStatus.STATE_FAILED
                message.detail = ("Shutdown has closed command admission" if self._closing else
                                  "Reset requires a latched, confirmed software stop"
                                  if reset else
                                  "Software stop is latched; this request was not executed")
                message.execution_idle = True
            self._statuses[request.request_id] = deepcopy(message)
            return not blocked, message

    def request_stop(self, request):
        """Install fences before acknowledging a stop, including unseen targets."""
        with self._lock:
            if request.execution_instance_id != self.execution_instance_id:
                return False
            self._stopping = True
            targets = set()
            instances = request.stop_execution_instance_ids
            if len(instances) != len(request.stop_request_ids):
                self._foreign_targets.update(request.stop_request_ids)
            else:
                for request_id, instance_id in zip(request.stop_request_ids, instances):
                    if instance_id == self.execution_instance_id:
                        targets.add(request_id)
                    else:
                        self._foreign_targets.add(request_id)
            recovery_allowed = not self._foreign_targets
            self._active_stop_id = request.request_id if recovery_allowed else None
            targets.add(request.request_id)
            targets.update(
                key for key, value in self._statuses.items()
                if not value.execution_idle
            )
            self._fenced.update(targets)
            targets.update(self._foreign_targets)
            for request_id in targets:
                previous = self._statuses.get(request_id)
                message = deepcopy(previous) if previous is not None else CommandStatus()
                message.request_id = request_id
                message.execution_instance_id = self.execution_instance_id
                message.state = (CommandStatus.STATE_STOP_REQUESTED if recovery_allowed
                                 else CommandStatus.STATE_STOP_UNCONFIRMED)
                message.detail = ("BT stop requested; execution is not yet confirmed idle"
                                  if recovery_allowed else
                                  "Prior execution instance owns unresolved work; recovery is blocked")
                message.execution_idle = False
                message.buffered_command_count = 0
                message.retry_safe = False
                message.continuation_safe = False
                self._statuses[request_id] = message
            return recovery_allowed

    def record(self, message):
        if not message.request_id:
            return deepcopy(message)
        with self._lock:
            previous = self._statuses.get(message.request_id)
            if self._foreign_targets and message.request_id in self._fenced | self._foreign_targets:
                return deepcopy(previous)
            if previous is not None and previous.execution_idle:
                return deepcopy(previous)
            if (message.request_id in self._fenced
                    and message.state not in {
                        CommandStatus.STATE_STOP_REQUESTED,
                        CommandStatus.STATE_STOP_UNCONFIRMED,
                        CommandStatus.STATE_STOP_CONFIRMED,
                    }):
                return deepcopy(previous)
            result = deepcopy(message)
            result.execution_instance_id = self.execution_instance_id
            self._statuses[message.request_id] = result
            return deepcopy(result)

    def confirm_stop(self, message):
        """Confirm every fenced BT identity only after global owner quiescence."""
        if message.state != CommandStatus.STATE_STOP_CONFIRMED or not message.execution_idle:
            raise ValueError("Stop completion requires explicit idle evidence")
        with self._lock:
            if self._foreign_targets or message.request_id != self._active_stop_id:
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
            self._active_stop_id = None
            return confirmed

    @property
    def stop_latched(self):
        with self._lock:
            return self._stopping

    def reset_stop(self, request_id, reset_resources, *, clear_admission=None):
        """Clear admission only after retained and freshly measured idle proof."""
        with self._lock:
            if (self._closing or not self._stopping or self._foreign_targets
                    or any(not item.execution_idle for key, item in self._statuses.items()
                           if key != request_id)):
                return False, "Software stop ownership is unresolved; reset remains blocked"
            def clear_latch():
                if clear_admission is not None:
                    clear_admission()
                self._stopping = False
            return reset_resources(clear_latch)

    def contains(self, request_id):
        with self._lock:
            return request_id in self._statuses

    def query(self, request_id):
        with self._lock:
            if request_id in self._statuses:
                return deepcopy(self._statuses[request_id])
        return self.unknown_status(request_id, "BT owner has no retained evidence for this request")

    def query_snapshot(self, request_id, expected_instance_id):
        """Return a fresh heartbeat and occupancy without adopting any request."""
        with self._lock:
            if ((request_id or expected_instance_id)
                    and expected_instance_id != self.execution_instance_id):
                status = self.unknown_status(request_id, "Execution instance changed; prior work is unresolved")
            else:
                status = self.query(request_id)
                if not request_id and self._foreign_targets:
                    status.state = CommandStatus.STATE_STOP_UNCONFIRMED
                    status.detail = "Prior shutdown stop-unconfirmed; reconciliation is required: " + "; ".join(
                        self._statuses[key].detail for key in sorted(self._foreign_targets)
                    )
            unresolved = sorted(key for key, value in self._statuses.items()
                                if not value.execution_idle)
            accepting = self._activated and not self._stopping and not unresolved
            return self.outgoing_status(status), accepting, unresolved
