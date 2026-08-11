"""Shared setup coordination contracts."""

from .setup_context import (
    SetupContextEvent,
    SetupContextLifecycle,
    SetupContextSnapshot,
    StaleSetupContext,
    new_context_id,
    validate_context_id,
)
from .setup_coordinator import (
    SetupCoordinator,
    SetupOperation,
    SetupOperationStatus,
)

__all__ = [
    "SetupContextEvent",
    "SetupContextLifecycle",
    "SetupContextSnapshot",
    "SetupCoordinator",
    "SetupOperation",
    "SetupOperationStatus",
    "StaleSetupContext",
    "new_context_id",
    "validate_context_id",
]
