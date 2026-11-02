"""Bound accepted work before an executor schedules its yielding callback."""

from dataclasses import dataclass
from threading import RLock


@dataclass
class _Reservation:
    context_id: str
    lane: str
    claimed: bool = False
    cancelled: bool = False


class OperationAdmission:
    """Reserve capacity by request, context and physical lane until completion."""

    def __init__(self, limit=1):
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
            raise ValueError("Operation limit must be a positive integer")
        self._limit = limit
        self._lock = RLock()
        self._reservations = {}
        self._closed = False

    def reserve(self, key, *, context_id="", lane=""):
        with self._lock:
            if self._closed:
                return False
            if key in self._reservations:
                item = self._reservations[key]
                return (item.context_id, item.lane) == (context_id, lane)
            if len(self._reservations) >= self._limit:
                return False
            if any((context_id and context_id == item.context_id)
                   or (lane and lane == item.lane)
                   for item in self._reservations.values()):
                return False
            self._reservations[key] = _Reservation(context_id, lane)
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
