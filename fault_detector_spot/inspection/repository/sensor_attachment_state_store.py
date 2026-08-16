"""Persist the selected physical inspection sensor."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    sensor_probe_frame,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
)
from fault_detector_spot.shared.persistence.runtime_paths import (
    fault_detector_runtime_root,
)


@dataclass(frozen=True)
class PersistedSensorAttachmentSelection:
    """Store the selected sensor identity and attachment revision."""

    sensor_id: str
    attachment_revision: int

    @classmethod
    def from_dict(
        cls,
        data: Dict[str, Any],
    ) -> "PersistedSensorAttachmentSelection":
        """Create persisted attachment selection from YAML data."""
        if not isinstance(data, dict):
            raise ValueError("Sensor attachment state must be an object")
        selection = cls(
            sensor_id=str(data["sensor_id"]),
            attachment_revision=int(data["attachment_revision"]),
        )
        selection.validate()
        return selection

    def validate(self) -> None:
        """Validate persisted attachment selection."""
        if self.sensor_id:
            sensor_probe_frame(self.sensor_id)
        if (
            isinstance(self.attachment_revision, bool)
            or not isinstance(self.attachment_revision, int)
            or self.attachment_revision < 0
        ):
            raise ValueError(
                "Attachment revision must be a non-negative integer"
            )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize persisted attachment selection."""
        return {
            "sensor_id": self.sensor_id,
            "attachment_revision": self.attachment_revision,
        }


class SensorAttachmentStateStore:
    """Persist selection without persisting physical confirmation."""

    def __init__(
        self,
        path: Optional[Union[str, Path]] = None,
    ):
        """Create the store under the persistent runtime root."""
        if path is None:
            path = (
                fault_detector_runtime_root()
                / "sensor_attachment_selection.yaml"
            )
        self.path = Path(path).expanduser()

    def load(
        self,
    ) -> Optional[PersistedSensorAttachmentSelection]:
        """Load the last selection or return None on first startup."""
        if not self.path.is_file():
            return None
        try:
            with self.path.open("r", encoding="utf-8") as state_file:
                data = yaml.safe_load(state_file)
        except yaml.YAMLError as exception:
            raise ValueError(
                f"Invalid sensor attachment YAML in {self.path}: "
                f"{exception}"
            ) from exception
        return PersistedSensorAttachmentSelection.from_dict(data)

    def clear(self) -> None:
        """Remove persisted selection so bare hand remains the default."""
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass

    def save(
        self,
        selection: PersistedSensorAttachmentSelection,
    ) -> PersistedSensorAttachmentSelection:
        """Persist only selected identity and revision."""
        selection.validate()
        content = yaml.safe_dump(
            selection.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(self.path, content)
        return selection
