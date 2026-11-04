"""Bounded shutdown while the caller continues servicing ROS callbacks."""

from concurrent.futures import Future
from contextlib import contextmanager
from dataclasses import dataclass
import math
import signal
from threading import Event, Lock, Thread
import time


@dataclass(frozen=True)
class ShutdownReport:
    confirmed: bool
    unresolved: dict
    cleanup_errors: dict


def _start(operation):
    """Isolate a stuck owner/cleanup without an unbounded thread-pool join."""
    future = Future()

    def run():
        try:
            future.set_result(operation())
        except Exception as error:
            future.set_exception(error)

    Thread(target=run, name="shutdown-operation", daemon=True).start()
    return future


class ShutdownCoordinator:
    """Close admission, detach owners, drain local work, then tear down.

    Operations must not start new motion. ``progress`` runs on the calling
    thread and services callbacks for at most the supplied timeout. Individual
    owner calls run separately so a busy owner cannot prevent detachment of
    other resources or defeat the deadline. Failed cleanup is retained for a
    later close; an operation still running is never invoked twice.
    """

    def __init__(self, *, close_admission, request_stop, confirmations, cleanup,
                 progress, report_unconfirmed, timeout_sec=10.0,
                 cleanup_timeout_sec=20.0, monotonic_clock=time.monotonic,
                 passive_confirmations=None, retained_unresolved=None):
        for value in (timeout_sec, cleanup_timeout_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Shutdown deadlines must be finite and positive")
        self._admission = dict(close_admission)
        self._requests = dict(request_stop)
        self._confirmations = dict(confirmations)
        self._passive_confirmations = dict(passive_confirmations or {})
        self._retained_unresolved = dict(retained_unresolved or {})
        self._cleanup = dict(cleanup)
        self._progress = progress
        self._report_unconfirmed = report_unconfirmed
        self._timeout = timeout_sec
        self._cleanup_timeout = cleanup_timeout_sec
        self._clock = monotonic_clock
        self._close_lock = Lock()
        self._confirmation_complete = False
        self._confirmed = False
        self.unresolved = {name: "Shutdown ownership check has not completed"
                           for name in (*confirmations, *self._passive_confirmations)}
        self.unresolved.update(self._retained_unresolved)
        self.cleanup_errors = {}
        self._cleanup_running = {}
        self._cleanup_done = set()

    def _advance(self, deadline):
        remaining = deadline - self._clock()
        if remaining <= 0:
            return
        interval = min(0.02, remaining)
        try:
            self._progress(interval)
        except Exception:
            # An unavailable executor is not completion evidence. Polls remain
            # unresolved and the same deadline still applies.
            Event().wait(interval)
        # Yield to owner workers even for a fake/nonblocking progress function.
        time.sleep(0)

    def _wait(self, future, deadline):
        while not future.done() and self._clock() < deadline:
            self._advance(deadline)
        return future.done()

    def _confirm(self):
        deadline = self._clock() + self._timeout
        for name, operation in self._admission.items():
            future = _start(operation)
            if not self._wait(future, deadline):
                self.unresolved[f"admission/{name}"] = "Admission closure timed out"
                continue
            try:
                future.result()
            except Exception as error:
                self.unresolved[f"admission/{name}"] = str(error)

        requests = {name: _start(operation)
                    for name, operation in self._requests.items()}
        polls = {}
        snapshots = None
        sweep_started = None
        while self._clock() < deadline:
            for name, future in tuple(requests.items()):
                if not future.done():
                    continue
                try:
                    future.result()
                except Exception as error:
                    self.unresolved[f"request/{name}"] = f"Shutdown request failed: {error}"
                del requests[name]
            if requests:
                self._advance(deadline)
                continue
            if snapshots is None:
                snapshots = {name: _start(operation)
                             for name, operation in self._passive_confirmations.items()}
            for name, future in tuple(snapshots.items()):
                if not future.done():
                    continue
                try:
                    confirmed, detail = future.result()
                except Exception as error:
                    confirmed, detail = False, f"Ownership snapshot failed: {error}"
                if confirmed:
                    self.unresolved.pop(name, None)
                else:
                    self.unresolved[name] = detail or "Detached work remains unresolved"
                del snapshots[name]
            if snapshots:
                self._advance(deadline)
                continue
            if not polls:
                # Revalidate local drains together: a callback can still be
                # completing while another owner reports idle.
                sweep_started = self._clock()
                polls = {name: _start(operation)
                         for name, operation in self._confirmations.items()}
            if polls and all(future.done() for future in polls.values()):
                for name, future in polls.items():
                    try:
                        confirmed, detail = future.result()
                    except Exception as error:
                        confirmed, detail = False, f"Shutdown drain failed: {error}"
                    if confirmed:
                        self.unresolved.pop(name, None)
                    else:
                        self.unresolved[name] = detail or "Local shutdown drain remains incomplete"
                polls = {}
                # A blocked poll may return much later than its peers. Demand
                # another complete fresh sweep before using that mixed proof.
                fresh = self._clock() - sweep_started <= 0.1
                # Historical or detached work cannot become confirmed during
                # teardown. Preserve it in the report without waiting for it.
                waiting = self.unresolved.keys() & self._confirmations.keys()
                if not waiting and fresh:
                    break
            elif not polls:
                break
            self._advance(deadline)
        else:
            # Even formerly confirmed participants need a current complete
            # sweep. Never turn a timed-out revalidation into stopped state.
            for name, future in polls.items():
                if not future.done():
                    self.unresolved[name] = "Local shutdown drain did not finish before deadline"
            for name in snapshots or {}:
                self.unresolved[name] = "Ownership snapshot did not finish before shutdown deadline"
            if not self.unresolved:
                self.unresolved["confirmation"] = "Local shutdown drain deadline expired"
        for name in requests:
            self.unresolved[f"request/{name}"] = "Shutdown request did not finish before deadline"
        self._confirmed = not self.unresolved
        # Freeze the outcome before releasing any resource. Later close calls
        # may retry cleanup, but cannot reinterpret discarded evidence as idle.
        self._confirmation_complete = True
        if self.unresolved:
            # Reporting uses the same bounded, retryable cleanup path and runs
            # first, before any monitor or client is released.
            self._cleanup = {"report stop-unconfirmed": lambda: self._report_unconfirmed(
                dict(self.unresolved)), **self._cleanup}

    def close(self):
        if not self._close_lock.acquire(blocking=False):
            return ShutdownReport(False, dict(self.unresolved), dict(self.cleanup_errors))
        try:
            if not self._confirmation_complete:
                self._confirm()
            deadline = self._clock() + self._cleanup_timeout
            remaining_names = [name for name in self._cleanup
                               if name not in self._cleanup_done]
            reserve = min(0.1, self._cleanup_timeout / max(1, len(remaining_names)))
            for index, name in enumerate(remaining_names):
                future = self._cleanup_running.get(name)
                if future is None:
                    future = _start(self._cleanup[name])
                    self._cleanup_running[name] = future
                # Leave a bounded opportunity for every independent resource.
                allowance = max(0.0, deadline - self._clock()
                                - reserve * (len(remaining_names) - index - 1))
                if not self._wait(future, self._clock() + allowance):
                    self.cleanup_errors[name] = "Cleanup did not finish before deadline"
                    continue
                del self._cleanup_running[name]
                try:
                    result = future.result()
                    if result is False:
                        raise RuntimeError("Cleanup reported incomplete")
                except Exception as error:
                    self.cleanup_errors[name] = str(error)
                else:
                    self._cleanup_done.add(name)
                    self.cleanup_errors.pop(name, None)
            return ShutdownReport(self._confirmed, dict(self.unresolved),
                                  dict(self.cleanup_errors))
        finally:
            self._close_lock.release()


@contextmanager
def shutdown_signals():
    """Request ordered shutdown without invalidating the ROS context first."""
    requested = Event()
    previous = {}
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: requested.set())
        yield requested
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
