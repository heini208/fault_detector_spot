"""Persistent metadata for one multi-channel probe measurement."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple

from fault_detector_spot.inspection.model.sensor_models import SensorChannel
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


class MeasurementCompletionState(str, Enum):
    """Lifecycle state persisted with a measurement recording."""

    RECORDING = "recording"
    COMPLETE = "complete"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class MeasurementRecording:
    """Self-contained metadata snapshot for one probe measurement."""

    object_id: str
    routine_id: str
    probe_point_id: str
    sensor_id: str
    attachment_revision: int
    started_at_ns: int
    configured_channels: Tuple[SensorChannel, ...]
    completion_state: MeasurementCompletionState = (
        MeasurementCompletionState.RECORDING
    )
    sample_counts: Mapping[str, int] = field(default_factory=dict)
    finished_at_ns: Optional[int] = None

    @classmethod
    def start(
        cls,
        *,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        sensor_id: str,
        attachment_revision: int,
        started_at_ns: int,
        configured_channels: Tuple[SensorChannel, ...],
    ) -> "MeasurementRecording":
        """Create validated initial metadata with zero sample counts."""
        channels = tuple(configured_channels)
        recording = cls(
            object_id=object_id,
            routine_id=routine_id,
            probe_point_id=probe_point_id,
            sensor_id=sensor_id,
            attachment_revision=attachment_revision,
            started_at_ns=started_at_ns,
            configured_channels=channels,
            sample_counts={
                channel.channel_id: 0
                for channel in channels
            },
        )
        recording.validate()
        return recording

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MeasurementRecording":
        """Restore and validate persisted measurement metadata."""
        if not isinstance(data, dict):
            raise ValueError("measurement metadata must be an object")
        raw_channels = data["configured_channels"]
        if not isinstance(raw_channels, list):
            raise ValueError("configured channels must be an array")
        raw_counts = data["sample_counts"]
        if not isinstance(raw_counts, dict):
            raise ValueError("sample counts must be an object")
        recording = cls(
            object_id=str(data["object_id"]),
            routine_id=str(data["routine_id"]),
            probe_point_id=str(data["probe_point_id"]),
            sensor_id=str(data["sensor_id"]),
            attachment_revision=data["attachment_revision"],
            started_at_ns=data["started_at_ns"],
            configured_channels=tuple(
                SensorChannel.from_dict(channel)
                for channel in raw_channels
            ),
            completion_state=MeasurementCompletionState(
                data["completion_state"]
            ),
            sample_counts=dict(raw_counts),
            finished_at_ns=data.get("finished_at_ns"),
        )
        recording.validate()
        return recording

    @property
    def identity(self) -> Tuple[str, str, str, str, int]:
        """Return the natural recording identity."""
        return (
            self.object_id,
            self.routine_id,
            self.probe_point_id,
            self.sensor_id,
            self.started_at_ns,
        )

    def validate(self) -> None:
        """Validate recording identity, lifecycle, and channel snapshot."""
        for value, label in (
            (self.object_id, "object ID"),
            (self.routine_id, "routine ID"),
            (self.probe_point_id, "probe point ID"),
            (self.sensor_id, "sensor ID"),
        ):
            validate_storage_name(value, label)
        if (
            isinstance(self.attachment_revision, bool)
            or not isinstance(self.attachment_revision, int)
            or self.attachment_revision < 0
        ):
            raise ValueError(
                "Attachment revision must be a non-negative integer"
            )
        self._validate_timestamp(self.started_at_ns, "Start timestamp")
        if not isinstance(
            self.completion_state,
            MeasurementCompletionState,
        ):
            raise TypeError(
                "Completion state must be a MeasurementCompletionState"
            )
        if not self.configured_channels:
            raise ValueError(
                "Measurement recording requires at least one channel"
            )
        channel_ids = []
        for channel in self.configured_channels:
            if not isinstance(channel, SensorChannel):
                raise TypeError(
                    "Configured channels must contain SensorChannel values"
                )
            channel.validate()
            channel_ids.append(channel.channel_id)
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError("Configured channel IDs must be unique")
        if not isinstance(self.sample_counts, Mapping):
            raise TypeError("Sample counts must be a mapping")
        if set(self.sample_counts) != set(channel_ids):
            raise ValueError(
                "Sample counts must match the configured channel IDs"
            )
        for count in self.sample_counts.values():
            if (
                isinstance(count, bool)
                or not isinstance(count, int)
                or count < 0
            ):
                raise ValueError(
                    "Sample counts must be non-negative integers"
                )
        if self.completion_state == MeasurementCompletionState.RECORDING:
            if self.finished_at_ns is not None:
                raise ValueError(
                    "An active recording must not have a finish timestamp"
                )
        else:
            self._validate_timestamp(
                self.finished_at_ns,
                "Finish timestamp",
            )
            if self.finished_at_ns < self.started_at_ns:
                raise ValueError(
                    "Finish timestamp must not precede start timestamp"
                )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize complete, historical recording metadata."""
        return {
            "object_id": self.object_id,
            "routine_id": self.routine_id,
            "probe_point_id": self.probe_point_id,
            "sensor_id": self.sensor_id,
            "attachment_revision": self.attachment_revision,
            "started_at_ns": self.started_at_ns,
            "finished_at_ns": self.finished_at_ns,
            "configured_channels": [
                channel.to_dict()
                for channel in self.configured_channels
            ],
            "completion_state": self.completion_state.value,
            "sample_counts": {
                channel.channel_id: self.sample_counts[channel.channel_id]
                for channel in self.configured_channels
            },
        }

    @staticmethod
    def _validate_timestamp(value, label: str) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
        ):
            raise ValueError(
                f"{label} must be non-negative integer nanoseconds"
            )
