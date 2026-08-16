"""Coordinate shared setup contexts and physical command delegation."""

from dataclasses import dataclass
from threading import RLock
from typing import Callable, Dict

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandControllerStatus,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextEvent,
    SetupContextLifecycle,
    SetupContextSnapshot,
    StaleSetupContext,
    new_context_id,
    setup_origin,
    validate_context_id,
)


@dataclass(frozen=True)
class SetupOperation:
    """Bind one non-recordable physical command to a context revision."""

    context: SetupContextSnapshot
    request: CommandRequest[SemanticCommand]

    @property
    def request_id(self) -> str:
        return self.request.request_id


@dataclass(frozen=True)
class SetupOperationStatus:
    """Expose one current setup operation transition."""

    operation: SetupOperation
    state: CommandControllerState
    detail: str
    buffered_command_count: int


ContextListener = Callable[[SetupContextEvent], None]
OperationListener = Callable[[SetupOperationStatus], None]


class SetupCoordinator:
    """Own setup context lifecycle and shared command-controller access."""

    def __init__(self, command_controller: CommandController):
        self._command_controller = command_controller
        self._lock = RLock()
        self._contexts: Dict[str, SetupContextSnapshot] = {}
        self._prepared: Dict[str, SetupOperation] = {}
        self._operations: Dict[str, SetupOperation] = {}
        self._context_listeners = []
        self._operation_listeners: Dict[str, list] = {}
        self._command_controller.add_status_listener(
            self._handle_command_status
        )

    @property
    def contexts(self):
        with self._lock:
            return tuple(self._contexts.values())

    def contexts_for(self, origin) -> tuple:
        """Return current contexts owned by one setup domain."""
        expected_origin = setup_origin(origin)
        with self._lock:
            return tuple(
                context
                for context in self._contexts.values()
                if context.origin is expected_origin
            )

    def resolve_context(
        self,
        context_id: str,
        client_id: str,
        origin=None,
    ) -> SetupContextSnapshot:
        """Resolve one current context and enforce ownership."""
        normalized_context_id = validate_context_id(context_id)
        normalized_client_id = required_client_id(client_id)
        expected_origin = (
            setup_origin(origin)
            if origin is not None
            else None
        )
        with self._lock:
            context = self._contexts.get(normalized_context_id)
            if (
                context is None
                or (
                    expected_origin is not None
                    and context.origin is not expected_origin
                )
            ):
                raise LookupError(
                    f"Unknown setup context: {normalized_context_id}"
                )
            self._require_current_locked(context)
        if context.client_id != normalized_client_id:
            raise ValueError(
                "Client ID does not own the setup context"
            )
        return context

    def open_context(
        self,
        origin,
        client_id: str,
    ) -> SetupContextSnapshot:
        context = SetupContextSnapshot(
            context_id=new_context_id(),
            client_id=client_id,
            origin=setup_origin(origin),
            revision=0,
        )
        with self._lock:
            self._contexts[context.context_id] = context
        self._emit_context(
            context,
            SetupContextLifecycle.OPENED,
        )
        return context

    def advance_context(
        self,
        context: SetupContextSnapshot,
    ) -> SetupContextSnapshot:
        with self._lock:
            self._require_current_locked(context)
            self._discard_context_requests_locked(
                context.context_id
            )
            updated = SetupContextSnapshot(
                context_id=context.context_id,
                client_id=context.client_id,
                origin=context.origin,
                revision=context.revision + 1,
            )
            self._contexts[context.context_id] = updated
        self._emit_context(
            updated,
            SetupContextLifecycle.UPDATED,
        )
        return updated

    def replace_context(
        self,
        context: SetupContextSnapshot,
    ) -> SetupContextSnapshot:
        self.close_context(context)
        return self.open_context(
            context.origin,
            context.client_id,
        )

    def close_context(
        self,
        context: SetupContextSnapshot,
    ) -> None:
        with self._lock:
            self._require_current_locked(context)
            self._contexts.pop(context.context_id)
            self._discard_context_requests_locked(
                context.context_id
            )
            self._operation_listeners.pop(
                context.context_id,
                None,
            )
        self._emit_context(
            context,
            SetupContextLifecycle.CLOSED,
        )

    def is_current(
        self,
        context: SetupContextSnapshot,
    ) -> bool:
        with self._lock:
            return (
                self._contexts.get(context.context_id)
                == context
            )

    def require_current(
        self,
        context: SetupContextSnapshot,
    ) -> SetupContextSnapshot:
        with self._lock:
            self._require_current_locked(context)
        return context

    def prepare_command(
        self,
        context: SetupContextSnapshot,
        command: SemanticCommand,
    ) -> SetupOperation:
        if not isinstance(command, SemanticCommand):
            raise TypeError("Expected a SemanticCommand")
        with self._lock:
            self._require_current_locked(context)
            request = CommandRequest.create(
                command=command,
                client_id=context.client_id,
                context_id=context.context_id,
                origin=context.origin,
                recording_policy=RecordingPolicy.EXCLUDE,
            )
            operation = SetupOperation(
                context=context,
                request=request,
            )
            self._prepared[operation.request_id] = operation
        return operation

    def submit(self, operation: SetupOperation) -> str:
        if not isinstance(operation, SetupOperation):
            raise TypeError("Expected a SetupOperation")
        with self._lock:
            prepared = self._prepared.get(
                operation.request_id
            )
            if prepared != operation:
                raise ValueError(
                    "Operation was not prepared by this coordinator"
                )
            self._require_current_locked(operation.context)
            self._operations[
                operation.request_id
            ] = operation
            self._prepared.pop(operation.request_id)
        try:
            self._command_controller.submit(operation.request)
        except Exception:
            with self._lock:
                self._operations.pop(
                    operation.request_id,
                    None,
                )
            raise
        return operation.request_id

    def require_command_lane_idle(
        self,
        detail: str,
        queued_detail: str = "",
    ) -> None:
        """Reject work that requires an idle physical command lane."""
        message = detail.strip()
        if not message:
            raise ValueError(
                "Command-lane admission detail must not be empty"
            )
        queued_message = queued_detail.strip() or message
        if self._command_controller.active_request_id:
            raise RuntimeError(message)
        if self._command_controller.queued_request_ids:
            raise RuntimeError(queued_message)

    def cancel_operation(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> str:
        """Cancel one submitted operation owned by the current context."""
        normalized = validate_request_id(request_id)
        with self._lock:
            self._require_current_locked(context)
            operation = self._operations.get(normalized)
            if operation is None or operation.context != context:
                raise LookupError(
                    f"Unknown setup operation: {normalized}"
                )
        return self._command_controller.cancel(normalized)

    def add_context_listener(
        self,
        listener: ContextListener,
    ) -> None:
        self._add_listener(
            self._context_listeners,
            listener,
        )

    def remove_context_listener(
        self,
        listener: ContextListener,
    ) -> None:
        self._remove_listener(
            self._context_listeners,
            listener,
        )

    def add_operation_listener(
        self,
        context: SetupContextSnapshot,
        listener: OperationListener,
    ) -> None:
        if not callable(listener):
            raise TypeError("Listener must be callable")
        with self._lock:
            self._require_current_locked(context)
            listeners = self._operation_listeners.setdefault(
                context.context_id,
                [],
            )
            if listener not in listeners:
                listeners.append(listener)

    def remove_operation_listener(
        self,
        context: SetupContextSnapshot,
        listener: OperationListener,
    ) -> None:
        with self._lock:
            listeners = self._operation_listeners.get(
                context.context_id
            )
            if listeners is None:
                return
            if listener in listeners:
                listeners.remove(listener)
            if not listeners:
                self._operation_listeners.pop(
                    context.context_id,
                    None,
                )

    def _handle_command_status(
        self,
        status: CommandControllerStatus,
    ) -> None:
        with self._lock:
            operation = self._operations.get(
                status.request_id
            )
            current = (
                operation is not None
                and self._contexts.get(
                    operation.context.context_id
                )
                == operation.context
            )
            if status.state in {
                CommandControllerState.SUCCEEDED,
                CommandControllerState.FAILED,
                CommandControllerState.CANCELLED,
            }:
                self._operations.pop(
                    status.request_id,
                    None,
                )
        if not current:
            return
        setup_status = SetupOperationStatus(
            operation=operation,
            state=status.state,
            detail=status.detail or status.state.value,
            buffered_command_count=(
                status.buffered_command_count
            ),
        )
        with self._lock:
            listeners = tuple(
                self._operation_listeners.get(
                    operation.context.context_id,
                    (),
                )
            )
        for listener in listeners:
            listener(setup_status)

    def _emit_context(
        self,
        context: SetupContextSnapshot,
        lifecycle: SetupContextLifecycle,
    ) -> None:
        event = SetupContextEvent(
            context=context,
            lifecycle=lifecycle,
        )
        for listener in self._listener_snapshot(
            self._context_listeners
        ):
            listener(event)

    def _require_current_locked(
        self,
        context: SetupContextSnapshot,
    ) -> None:
        if not isinstance(context, SetupContextSnapshot):
            raise TypeError(
                "Expected a SetupContextSnapshot"
            )
        if (
            self._contexts.get(context.context_id)
            != context
        ):
            raise StaleSetupContext(
                "Setup context is closed or stale: "
                f"{context.context_id}"
            )

    def _discard_context_requests_locked(
        self,
        context_id: str,
    ) -> None:
        self._prepared = {
            request_id: operation
            for request_id, operation
            in self._prepared.items()
            if operation.context.context_id != context_id
        }
        self._operations = {
            request_id: operation
            for request_id, operation
            in self._operations.items()
            if operation.context.context_id != context_id
        }

    def _add_listener(self, listeners, listener) -> None:
        if not callable(listener):
            raise TypeError("Listener must be callable")
        with self._lock:
            if listener not in listeners:
                listeners.append(listener)

    def _remove_listener(self, listeners, listener) -> None:
        with self._lock:
            if listener in listeners:
                listeners.remove(listener)

    def _listener_snapshot(self, listeners):
        with self._lock:
            return tuple(listeners)

    def close(self) -> None:
        self._command_controller.remove_status_listener(
            self._handle_command_status
        )
        with self._lock:
            contexts = tuple(self._contexts.values())
            self._contexts.clear()
            self._prepared.clear()
            self._operations.clear()
        for context in contexts:
            self._emit_context(
                context,
                SetupContextLifecycle.CLOSED,
            )
        with self._lock:
            self._context_listeners.clear()
            self._operation_listeners.clear()


__all__ = [
    "SetupCoordinator",
    "SetupOperation",
    "SetupOperationStatus",
]
