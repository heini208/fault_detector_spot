"""Presentation models for physical sensor mounts."""

from dataclasses import dataclass
from enum import Enum


class SensorAttachmentViewStatus(str, Enum):
    """Presentation states for physical sensor attachment."""

    NONE = "none"
    PENDING = "pending"
    ACTIVE = "active"


class SensorHeadConnectionViewStatus(str, Enum):
    """Presentation states for a sensor head connected through the Agent."""

    UNKNOWN = "unknown"
    AGENT_UNAVAILABLE = "agent_unavailable"
    NO_HEADS = "no_heads"
    UNASSIGNED = "unassigned"
    MATCHED = "matched"
    MISMATCH = "mismatch"


@dataclass(frozen=True)
class SensorDefinitionView:
    """Presentation data for one registered physical sensor."""

    sensor_id: str
    display_name: str
    probe_frame: str
    position: tuple
    orientation: tuple
    rotation_degrees: tuple
    channels: tuple = ()


@dataclass(frozen=True)
class SensorChannelView:
    """Presentation data for one configured measurement channel."""

    channel_id: str
    topic: str
    message_type: str


@dataclass(frozen=True)
class SensorTopicSuggestion:
    """One live ROS topic and its advertised message types."""

    topic: str
    message_types: tuple


@dataclass(frozen=True)
class SensorAttachmentView:
    """Presentation data for authoritative attachment state."""

    status: SensorAttachmentViewStatus
    active_sensor_id: str
    pending_sensor_id: str
    attachment_revision: int

    @property
    def selected_sensor_id(self) -> str:
        """Return the pending or active physical sensor ID."""
        return self.pending_sensor_id or self.active_sensor_id


@dataclass(frozen=True)
class SensorHeadConnectionView:
    """Presentation data for discovered sensor heads and ID matching."""

    status: SensorHeadConnectionViewStatus
    expected_sensor_id: str
    connected_sensor_ids: tuple
    detail: str


__all__ = [
    "SensorAttachmentView",
    "SensorAttachmentViewStatus",
    "SensorChannelView",
    "SensorDefinitionView",
    "SensorHeadConnectionView",
    "SensorHeadConnectionViewStatus",
    "SensorTopicSuggestion",
]
