"""Application command request contracts."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Generic, TypeVar

from .request_identity import new_request_id, validate_request_id


CommandT = TypeVar("CommandT")


class CommandOrigin(IntEnum):
    """Source responsible for one semantic command request."""

    UNSPECIFIED = 0
    OPERATIONAL = 1
    PROBE_SETUP = 2
    NAVIGATION_SETUP = 3
    PLAYBACK = 4
    INTERNAL = 5
    SYSTEM = 6


class RecordingPolicy(IntEnum):
    """Recorder treatment for one accepted semantic command."""

    UNSPECIFIED = 0
    EXCLUDE = 1
    INCLUDE_IF_RECORDING_ACTIVE = 2


_NON_RECORDABLE_ORIGINS = frozenset({
    CommandOrigin.PROBE_SETUP,
    CommandOrigin.NAVIGATION_SETUP,
    CommandOrigin.INTERNAL,
})


def _required_text(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{label} must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _optional_text(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{label} must be a string")
    return value.strip()


def _enum_value(value, enum_type, label):
    try:
        normalized = enum_type(value)
    except (TypeError, ValueError) as exception:
        raise ValueError(f"Unsupported {label}: {value!r}") from exception
    if normalized.value == 0:
        raise ValueError(f"{label} must be specified")
    return normalized


@dataclass(frozen=True)
class CommandRequest(Generic[CommandT]):
    """Validated semantic command plus dispatch and recording metadata."""

    request_id: str
    client_id: str
    context_id: str
    origin: CommandOrigin
    recording_policy: RecordingPolicy
    command: CommandT

    def __post_init__(self) -> None:
        request_id = validate_request_id(self.request_id)
        client_id = _required_text(self.client_id, "Client ID")
        context_id = _optional_text(self.context_id, "Context ID")
        origin = _enum_value(self.origin, CommandOrigin, "command origin")
        recording_policy = _enum_value(
            self.recording_policy,
            RecordingPolicy,
            "recording policy",
        )
        if self.command is None:
            raise ValueError("Command must not be None")
        if (
            origin in _NON_RECORDABLE_ORIGINS
            and recording_policy is not RecordingPolicy.EXCLUDE
        ):
            raise ValueError(f"Commands from {origin.name} must be excluded")
        object.__setattr__(self, "request_id", request_id)
        object.__setattr__(self, "client_id", client_id)
        object.__setattr__(self, "context_id", context_id)
        object.__setattr__(self, "origin", origin)
        object.__setattr__(self, "recording_policy", recording_policy)

    @classmethod
    def create(
        cls,
        command: CommandT,
        client_id: str,
        origin: CommandOrigin,
        recording_policy: RecordingPolicy,
        context_id: str = "",
    ) -> "CommandRequest[CommandT]":
        """Create one request with a new correlation identity."""
        return cls(
            request_id=new_request_id(),
            client_id=client_id,
            context_id=context_id,
            origin=origin,
            recording_policy=recording_policy,
            command=command,
        )
