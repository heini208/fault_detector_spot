"""Presentation models for physical sensor mounts."""

from dataclasses import dataclass
from enum import Enum


class SensorAttachmentViewStatus(str, Enum):
    """Presentation states for physical sensor attachment."""

    NONE = "none"
    PENDING = "pending"
    ACTIVE = "active"


@dataclass(frozen=True)
class SensorDefinitionView:
    """Presentation data for one registered physical sensor."""

    sensor_id: str
    display_name: str
    probe_frame: str
    position: tuple
    orientation: tuple
    rotation_degrees: tuple


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


__all__ = [
    "SensorAttachmentView",
    "SensorAttachmentViewStatus",
    "SensorDefinitionView",
]

