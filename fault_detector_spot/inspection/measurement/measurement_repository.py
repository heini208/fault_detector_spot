"""Exclusive JSONL persistence for generic sensor measurements."""

import json
import os
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, TextIO, Tuple, Union

from fault_detector_spot.inspection.measurement.measurement_models import (
    MeasurementCompletionState,
    MeasurementRecording,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


@dataclass
class _OpenRecording:
    recording: MeasurementRecording
    channel_files: Dict[str, TextIO]
    sample_counts: Dict[str, int]


class MeasurementRepository:
    """Own measurement paths, open channel files, and metadata."""

    CHANNEL_SUFFIX = ".jsonl"
    METADATA_SUFFIX = ".metadata.json"

    def __init__(
        self,
        root_dir: Optional[Union[str, Path]] = None,
    ):
        """Create the repository under ROS_HOME by default."""
        if root_dir is None:
            ros_home = Path(
                os.environ.get(
                    "ROS_HOME",
                    str(Path.home() / ".ros"),
                )
            )
            root_dir = ros_home / "fault_detector_spot" / "measurements"
        self.root_dir = Path(root_dir).expanduser()
        self._open_recordings: Dict[
            Tuple[str, str, str, str, int],
            _OpenRecording,
        ] = {}

    def get_channel_path(
        self,
        recording: MeasurementRecording,
        channel_id: str,
    ) -> Path:
        """Return the canonical JSONL path for one configured channel."""
        recording.validate()
        validate_storage_name(channel_id, "channel ID")
        if channel_id not in {
            channel.channel_id
            for channel in recording.configured_channels
        }:
            raise KeyError(f"Unknown configured channel: {channel_id}")
        return (
            self._sensor_directory(recording)
            / channel_id
            / f"{self._timestamp_name(recording.started_at_ns)}"
            f"{self.CHANNEL_SUFFIX}"
        )

    def get_metadata_path(self, recording: MeasurementRecording) -> Path:
        """Return the sidecar path for complete recording metadata."""
        recording.validate()
        return (
            self._sensor_directory(recording)
            / f"{self._timestamp_name(recording.started_at_ns)}"
            f"{self.METADATA_SUFFIX}"
        )

    def create(
        self,
        recording: MeasurementRecording,
    ) -> MeasurementRecording:
        """Exclusively create every channel file and initial metadata."""
        recording.validate()
        if recording.completion_state != MeasurementCompletionState.RECORDING:
            raise ValueError("A new measurement must be in recording state")
        if any(recording.sample_counts.values()):
            raise ValueError("A new measurement must have zero sample counts")
        if recording.identity in self._open_recordings:
            raise RuntimeError("Measurement recording is already open")

        created_paths = []
        channel_files = {}
        try:
            for channel in recording.configured_channels:
                path = self.get_channel_path(recording, channel.channel_id)
                path.parent.mkdir(parents=True, exist_ok=True)
                channel_files[channel.channel_id] = path.open(
                    "x",
                    encoding="utf-8",
                )
                created_paths.append(path)
            metadata_path = self.get_metadata_path(recording)
            self._write_exclusive_metadata(metadata_path, recording)
            created_paths.append(metadata_path)
        except Exception:
            for channel_file in channel_files.values():
                channel_file.close()
            for path in created_paths:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            raise

        self._open_recordings[recording.identity] = _OpenRecording(
            recording=recording,
            channel_files=channel_files,
            sample_counts=dict(recording.sample_counts),
        )
        return recording

    def append_sample(
        self,
        recording: MeasurementRecording,
        channel_id: str,
        sample: Mapping[str, Any],
    ) -> None:
        """Append one JSON-compatible sample without per-sample fsync."""
        context = self._require_open(recording)
        if channel_id not in context.channel_files:
            raise KeyError(f"Unknown configured channel: {channel_id}")
        if not isinstance(sample, Mapping):
            raise TypeError("Measurement sample must be an object")
        line = json.dumps(
            dict(sample),
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        context.channel_files[channel_id].write(line + "\n")
        context.sample_counts[channel_id] += 1

    def flush(self, recording: MeasurementRecording) -> None:
        """Flush and synchronize all open channel files."""
        context = self._require_open(recording)
        for channel_file in context.channel_files.values():
            channel_file.flush()
            os.fsync(channel_file.fileno())

    def finalize(
        self,
        recording: MeasurementRecording,
        completion_state: MeasurementCompletionState,
        finished_at_ns: int,
    ) -> MeasurementRecording:
        """Close channel files and atomically publish final metadata."""
        if completion_state == MeasurementCompletionState.RECORDING:
            raise ValueError("Final completion state must be terminal")
        context = self._require_open(recording)
        finalized = replace(
            recording,
            completion_state=completion_state,
            finished_at_ns=finished_at_ns,
            sample_counts=dict(context.sample_counts),
        )
        finalized.validate()

        try:
            self.flush(recording)
        finally:
            for channel_file in context.channel_files.values():
                channel_file.close()
            self._open_recordings.pop(recording.identity, None)

        content = json.dumps(
            finalized.to_dict(),
            indent=2,
            ensure_ascii=False,
        )
        atomic_write_text(
            self.get_metadata_path(finalized),
            content + "\n",
        )
        return finalized

    def load(
        self,
        *,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        sensor_id: str,
        started_at_ns: int,
    ) -> MeasurementRecording:
        """Load one recording by its natural identity."""
        path = self._metadata_path_from_identity(
            object_id=object_id,
            routine_id=routine_id,
            probe_point_id=probe_point_id,
            sensor_id=sensor_id,
            started_at_ns=started_at_ns,
        )
        if not path.is_file():
            raise FileNotFoundError(
                f"Measurement metadata does not exist: {path}"
            )
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exception:
            raise ValueError(
                f"Invalid measurement metadata in {path}: {exception}"
            ) from exception
        recording = MeasurementRecording.from_dict(data)
        expected_identity = (
            object_id,
            routine_id,
            probe_point_id,
            sensor_id,
            started_at_ns,
        )
        if recording.identity != expected_identity:
            raise ValueError(f"Measurement identity mismatch in {path}")
        return recording

    def is_open(self, recording: MeasurementRecording) -> bool:
        """Return whether this repository owns live files for recording."""
        return recording.identity in self._open_recordings

    def _require_open(
        self,
        recording: MeasurementRecording,
    ) -> _OpenRecording:
        context = self._open_recordings.get(recording.identity)
        if context is None or context.recording is not recording:
            raise RuntimeError("Measurement recording is not open")
        return context

    def _sensor_directory(
        self,
        recording: MeasurementRecording,
    ) -> Path:
        return (
            self.root_dir
            / recording.object_id
            / recording.routine_id
            / recording.probe_point_id
            / self._date_name(recording.started_at_ns)
            / recording.sensor_id
        )

    def _metadata_path_from_identity(
        self,
        *,
        object_id: str,
        routine_id: str,
        probe_point_id: str,
        sensor_id: str,
        started_at_ns: int,
    ) -> Path:
        for value, label in (
            (object_id, "object ID"),
            (routine_id, "routine ID"),
            (probe_point_id, "probe point ID"),
            (sensor_id, "sensor ID"),
        ):
            validate_storage_name(value, label)
        MeasurementRecording._validate_timestamp(
            started_at_ns,
            "Start timestamp",
        )
        return (
            self.root_dir
            / object_id
            / routine_id
            / probe_point_id
            / self._date_name(started_at_ns)
            / sensor_id
            / f"{self._timestamp_name(started_at_ns)}"
            f"{self.METADATA_SUFFIX}"
        )

    @staticmethod
    def _date_name(timestamp_ns: int) -> str:
        seconds = timestamp_ns // 1_000_000_000
        return datetime.fromtimestamp(
            seconds,
            tz=timezone.utc,
        ).strftime("%Y-%m-%d")

    @staticmethod
    def _timestamp_name(timestamp_ns: int) -> str:
        seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
        prefix = datetime.fromtimestamp(
            seconds,
            tz=timezone.utc,
        ).strftime("%Y-%m-%dT%H-%M-%S")
        return f"{prefix}.{nanoseconds:09d}Z"

    @staticmethod
    def _write_exclusive_metadata(
        path: Path,
        recording: MeasurementRecording,
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        content = json.dumps(
            recording.to_dict(),
            indent=2,
            ensure_ascii=False,
        )
        created = False
        try:
            with path.open("x", encoding="utf-8") as metadata_file:
                created = True
                metadata_file.write(content + "\n")
                metadata_file.flush()
                os.fsync(metadata_file.fileno())
        except Exception:
            if created:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            raise
