"""Shared context-scoped setup operation bookkeeping."""

from dataclasses import dataclass
from threading import RLock
from typing import Generic, TypeVar

from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)


PayloadT = TypeVar("PayloadT")


@dataclass(frozen=True)
class TrackedSetupOperation(Generic[PayloadT]):
    context: SetupContextSnapshot
    payload: PayloadT


class SetupOperationRegistry(Generic[PayloadT]):
    def __init__(self):
        self._lock = RLock()
        self._operations = {}
        self._listeners = []

    def register(
        self,
        request_id: str,
        context: SetupContextSnapshot,
        payload: PayloadT,
    ) -> TrackedSetupOperation[PayloadT]:
        request_id = validate_request_id(request_id)
        if not isinstance(context, SetupContextSnapshot):
            raise TypeError("Expected a SetupContextSnapshot")
        tracked = TrackedSetupOperation(
            context=context,
            payload=payload,
        )
        with self._lock:
            if request_id in self._operations:
                raise ValueError(
                    f"Setup request already tracked: {request_id}"
                )
            self._operations[request_id] = tracked
        return tracked

    def get(self, request_id: str):
        request_id = validate_request_id(request_id)
        with self._lock:
            return self._operations.get(request_id)

    def owned(
        self,
        request_id: str,
        context: SetupContextSnapshot,
    ):
        tracked = self.get(request_id)
        if tracked is None or tracked.context != context:
            return None
        return tracked

    def pop(self, request_id: str):
        request_id = validate_request_id(request_id)
        with self._lock:
            return self._operations.pop(request_id, None)

    def request_ids_for(
        self,
        context: SetupContextSnapshot,
    ) -> tuple:
        if not isinstance(context, SetupContextSnapshot):
            raise TypeError("Expected a SetupContextSnapshot")
        with self._lock:
            return tuple(
                request_id
                for request_id, tracked in self._operations.items()
                if tracked.context.context_id == context.context_id
            )

    def has_context(
        self,
        context: SetupContextSnapshot,
    ) -> bool:
        return bool(self.request_ids_for(context))

    def discard_context(
        self,
        context: SetupContextSnapshot,
    ) -> None:
        request_ids = self.request_ids_for(context)
        with self._lock:
            for request_id in request_ids:
                self._operations.pop(request_id, None)

    def add_listener(self, listener) -> None:
        if not callable(listener):
            raise TypeError("Listener must be callable")
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_listener(self, listener) -> None:
        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def emit(self, value) -> None:
        with self._lock:
            listeners = tuple(self._listeners)
        for listener in listeners:
            listener(value)

    def clear(self) -> None:
        with self._lock:
            self._operations.clear()
            self._listeners.clear()


__all__ = [
    "SetupOperationRegistry",
    "TrackedSetupOperation",
]
