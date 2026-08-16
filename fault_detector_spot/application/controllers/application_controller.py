"""Coordinate public operational intents with the command controller."""

from dataclasses import dataclass
from threading import RLock
from typing import Callable, Dict

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandControllerStatus,
    UnknownCommandRequest,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.application.setup.setup_context import SETUP_ORIGINS


@dataclass(frozen=True)
class ApplicationOperation:
    """Describe one validated operational request."""

    intent: int
    request: CommandRequest

    @property
    def request_id(self) -> str:
        """Return the correlated request identifier."""
        return self.request.request_id


@dataclass(frozen=True)
class ApplicationOperationStatus:
    """Expose one command-controller transition at application level."""

    operation: ApplicationOperation
    state: CommandControllerState
    detail: str
    buffered_command_count: int


class ApplicationController:
    """Own operational request construction, dispatch, and cancellation."""

    def __init__(self, command_controller: CommandController):
        self.command_controller = command_controller
        self.setup_coordinator = SetupCoordinator(command_controller)
        self.navigation_setup_coordinator = None
        self.probe_setup_coordinator = None
        self._lock = RLock()
        self._operations: Dict[str, ApplicationOperation] = {}
        self._status_listeners = []
        self.command_controller.add_status_listener(
            self._handle_command_status
        )

    def prepare_operation(
        self,
        intent: OperationalIntent,
        client_id: str,
        context_id: str = "",
    ) -> ApplicationOperation:
        """Validate public intent and create its semantic request."""
        self.validate_operation(intent, client_id)
        command = operational_intent_to_command(intent)
        request = CommandRequest.create(
            command=command,
            client_id=required_client_id(client_id),
            context_id=context_id.strip(),
            origin=CommandOrigin.OPERATIONAL,
            recording_policy=(
                RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
            ),
        )
        operation = ApplicationOperation(
            intent=int(intent.intent),
            request=request,
        )
        with self._lock:
            self._operations[operation.request_id] = operation
        return operation

    @staticmethod
    def validate_operation(
        intent: OperationalIntent,
        client_id: str,
    ) -> None:
        """Validate one public operation without creating request state."""
        required_client_id(client_id)
        operational_intent_to_command(intent)

    def submit(self, operation: ApplicationOperation) -> str:
        """Submit one operation through the serialized command lane."""
        with self._lock:
            prepared = self._operations.get(operation.request_id)
        if prepared != operation:
            raise ValueError("Operation was not prepared by this controller")
        try:
            self.command_controller.submit(operation.request)
        except Exception:
            with self._lock:
                self._operations.pop(operation.request_id, None)
            raise
        return operation.request_id

    def cancel(self, client_id: str, request_id: str) -> str:
        """Cancel one operation owned by the requesting client."""
        owner = required_client_id(client_id)
        with self._lock:
            operation = self._operations.get(request_id)
        if operation is None:
            raise UnknownCommandRequest(
                f"Unknown application operation: {request_id}"
            )
        if operation.request.client_id != owner:
            raise ValueError(
                "Client ID does not own the requested operation"
            )
        return self.command_controller.cancel(request_id)

    def emergency_stop(self, client_id: str) -> str:
        """Cancel all queued and active commands immediately."""
        return self.command_controller.cancel_all(
            required_client_id(client_id)
        )

    def add_status_listener(
        self,
        listener: Callable[[ApplicationOperationStatus], None],
    ) -> None:
        """Register one application-status listener."""
        if listener not in self._status_listeners:
            self._status_listeners.append(listener)

    def attach_navigation_setup(self, coordinator) -> None:
        """Attach the specialized navigation setup coordinator."""
        if not coordinator.uses_setup_coordinator(self.setup_coordinator):
            raise ValueError(
                "Navigation setup must use the shared setup coordinator"
            )
        if self.navigation_setup_coordinator is not None:
            raise RuntimeError("Navigation setup is already attached")
        self.navigation_setup_coordinator = coordinator

    def attach_probe_setup(self, coordinator) -> None:
        """Attach the specialized probe setup coordinator."""
        if not coordinator.uses_setup_coordinator(self.setup_coordinator):
            raise ValueError(
                "Probe setup must use the shared setup coordinator"
            )
        if self.probe_setup_coordinator is not None:
            raise RuntimeError("Probe setup is already attached")
        self.probe_setup_coordinator = coordinator

    def remove_status_listener(
        self,
        listener: Callable[[ApplicationOperationStatus], None],
    ) -> None:
        """Remove one application-status listener."""
        if listener in self._status_listeners:
            self._status_listeners.remove(listener)

    def _handle_command_status(
        self,
        status: CommandControllerStatus,
    ) -> None:
        with self._lock:
            operation = self._operations.get(status.request_id)
            tracked = operation is not None
        if operation is None and status.request.origin in SETUP_ORIGINS:
            return
        if operation is None:
            operation = ApplicationOperation(
                intent=OperationalIntent.INTENT_UNSPECIFIED,
                request=status.request,
            )
        application_status = ApplicationOperationStatus(
            operation=operation,
            state=status.state,
            detail=status.detail or status.state.value,
            buffered_command_count=status.buffered_command_count,
        )
        for listener in tuple(self._status_listeners):
            listener(application_status)
        if tracked and status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }:
            with self._lock:
                self._operations.pop(status.request_id, None)

    def close(self) -> None:
        """Detach from the command controller."""
        if self.probe_setup_coordinator is not None:
            self.probe_setup_coordinator.close()
            self.probe_setup_coordinator = None
        if self.navigation_setup_coordinator is not None:
            self.navigation_setup_coordinator.close()
            self.navigation_setup_coordinator = None
        self.setup_coordinator.close()
        self.command_controller.remove_status_listener(
            self._handle_command_status
        )
        with self._lock:
            self._operations.clear()
        self._status_listeners.clear()


__all__ = [
    "ApplicationController",
    "ApplicationOperation",
    "ApplicationOperationStatus",
]
