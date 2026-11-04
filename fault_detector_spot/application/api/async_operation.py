"""Track accepted commands and exclude conflicting setup workflows."""

from dataclasses import dataclass
from threading import RLock


@dataclass
class _Reservation:
    context_id: str
    lane: str
    queued_command: bool = False
    claimed: bool = False
    cancelled: bool = False


class OperationAdmission:
    """Keep accepted ownership until completion, sharing only queued commands."""

    def __init__(self, limit=1):
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
            raise ValueError("Operation limit must be a positive integer")
        self._limit = limit
        self._lock = RLock()
        self._reservations = {}
        self._closed = False

    def reserve(self, key, *, context_id="", lane=""):
        """Reserve exclusive workflow capacity by context and physical lane."""
        return self._reserve(key, context_id, lane, queued_command=False)

    def reserve_command(self, key, *, context_id=""):
        """Track a command whose execution is serialized by CommandController.

        Commands may share a context and wait in the controller's queue, but
        setup workflows remain exclusive with all accepted commands.
        """
        return self._reserve(key, context_id, "physical", queued_command=True)

    def _reserve(self, key, context_id, lane, *, queued_command):
        with self._lock:
            if self._closed:
                return False
            if key in self._reservations:
                item = self._reservations[key]
                return (item.context_id, item.lane, item.queued_command) == (
                    context_id, lane, queued_command,
                )
            if queued_command:
                if any(not item.queued_command for item in self._reservations.values()):
                    return False
            else:
                if len(self._reservations) >= self._limit:
                    return False
                if any(item.queued_command
                       or (context_id and context_id == item.context_id)
                       or (lane and lane == item.lane)
                       for item in self._reservations.values()):
                    return False
            self._reservations[key] = _Reservation(
                context_id, lane, queued_command=queued_command,
            )
            return True

    def claim(self, key):
        """Transfer accepted ownership to the executing callback exactly once."""
        with self._lock:
            item = self._reservations.get(key)
            if item is None or item.claimed or self._closed:
                return False
            item.claimed = True
            return True

    def cancel(self, key):
        with self._lock:
            item = self._reservations.get(key)
            if item is not None:
                item.cancelled = True

    def cancel_requested(self, key):
        with self._lock:
            item = self._reservations.get(key)
            return item is None or item.cancelled or self._closed

    def cancel_context(self, context_id):
        """Cancel active ownership and reclaim goals whose client left before execute."""
        with self._lock:
            for key, item in tuple(self._reservations.items()):
                if item.context_id == context_id:
                    item.cancelled = True
                    if not item.claimed:
                        self._reservations.pop(key)

    def release(self, key):
        with self._lock:
            self._reservations.pop(key, None)

    def release_pending(self, key):
        """Release a queued goal only after its rejected execute callback terminates."""
        with self._lock:
            item = self._reservations.get(key)
            if item is not None and not item.claimed:
                self._reservations.pop(key)

    def shutdown_status(self):
        """Accepted goals still need a terminal result even before execution starts."""
        with self._lock:
            if not self._reservations:
                return True, ""
            pending = sum(not item.claimed for item in self._reservations.values())
            contexts = sorted({item.context_id or "unspecified" for item in self._reservations.values()})
            return False, (
                f"Accepted operation completion pending: {pending} queued, "
                f"{len(self._reservations) - pending} executing; contexts: " + ", ".join(contexts)
            )

    def close(self):
        """Close admission while retaining every accepted goal until completion."""
        with self._lock:
            self._closed = True
            for item in self._reservations.values():
                item.cancelled = True
