"""Track API callbacks whose resources must survive shutdown confirmation."""

from functools import wraps
from inspect import iscoroutinefunction
from threading import RLock


def release_resources(owner, operations):
    """Release independent handles once; retain only failed work for retry."""
    lock = owner.__dict__.setdefault("_shutdown_cleanup_lock", RLock())
    with lock:
        completed = owner.__dict__.setdefault("_shutdown_cleanup_done", set())
        errors = []
        for name, release in operations.items():
            if name in completed:
                continue
            try:
                release()
            except Exception as error:
                errors.append(f"{name}: {error}")
            else:
                completed.add(name)
        if errors:
            raise RuntimeError("; ".join(errors))


class ShutdownCallbackActivity:
    def __init__(self):
        self._lock = RLock()
        self._active = 0

    def __enter__(self):
        with self._lock:
            self._active += 1

    def __exit__(self, *_exception):
        with self._lock:
            self._active -= 1

    def status(self):
        with self._lock:
            return self._active == 0, f"{self._active} API callbacks awaiting completion"


def tracked_shutdown_callback(callback):
    """Count through the terminal action transition, including async waits."""
    if iscoroutinefunction(callback):
        @wraps(callback)
        async def tracked(self, *args, **kwargs):
            with self._shutdown_callbacks:
                return await callback(self, *args, **kwargs)
    else:
        @wraps(callback)
        def tracked(self, *args, **kwargs):
            with self._shutdown_callbacks:
                return callback(self, *args, **kwargs)
    return tracked
