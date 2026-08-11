"""Immutable setup-context lifecycle contracts."""

from dataclasses import dataclass
from enum import Enum
from uuid import UUID, uuid4

from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.commanding.client_identity import (
    required_client_id,
)


SETUP_ORIGINS = frozenset({
    CommandOrigin.NAVIGATION_SETUP,
    CommandOrigin.PROBE_SETUP,
})


def new_context_id() -> str:
    """Return one canonical random setup-context identity."""
    return str(uuid4())


def validate_context_id(context_id: str) -> str:
    """Validate and normalize one canonical context UUID."""
    if not isinstance(context_id, str):
        raise TypeError("Context ID must be a string")
    value = context_id.strip()
    if not value:
        raise ValueError("Context ID must not be empty")
    try:
        parsed = UUID(value)
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError("Context ID must be a UUID") from exception
    canonical = str(parsed)
    if value != canonical:
        raise ValueError("Context ID must use canonical UUID form")
    return canonical


def setup_origin(value) -> CommandOrigin:
    """Return one supported setup origin."""
    try:
        origin = CommandOrigin(value)
    except (TypeError, ValueError) as exception:
        raise ValueError(f"Unsupported setup origin: {value!r}") from exception
    if origin not in SETUP_ORIGINS:
        raise ValueError(f"Unsupported setup origin: {origin.name}")
    return origin


@dataclass(frozen=True)
class SetupContextSnapshot:
    """Identify one immutable revision of an in-memory setup context."""

    context_id: str
    client_id: str
    origin: CommandOrigin
    revision: int

    def __post_init__(self) -> None:
        context_id = validate_context_id(self.context_id)
        client_id = required_client_id(self.client_id)
        origin = setup_origin(self.origin)
        if isinstance(self.revision, bool) or not isinstance(
            self.revision,
            int,
        ):
            raise TypeError("Context revision must be an integer")
        if self.revision < 0:
            raise ValueError("Context revision must not be negative")
        object.__setattr__(self, "context_id", context_id)
        object.__setattr__(self, "client_id", client_id)
        object.__setattr__(self, "origin", origin)


class SetupContextLifecycle(str, Enum):
    """Lifecycle transition of one setup context."""

    OPENED = "opened"
    UPDATED = "updated"
    CLOSED = "closed"


@dataclass(frozen=True)
class SetupContextEvent:
    """Publish one immutable context lifecycle transition."""

    context: SetupContextSnapshot
    lifecycle: SetupContextLifecycle


class StaleSetupContext(LookupError):
    """Raised when a context snapshot is closed or no longer current."""


__all__ = [
    "SETUP_ORIGINS",
    "SetupContextEvent",
    "SetupContextLifecycle",
    "SetupContextSnapshot",
    "StaleSetupContext",
    "new_context_id",
    "setup_origin",
    "validate_context_id",
]
