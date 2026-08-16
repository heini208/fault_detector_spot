"""Repository for hand-mounted sensor transform definitions."""

import os
from pathlib import Path
from typing import List, Optional, Union

import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


class SensorRepository:
    """Store editable physical sensor definitions."""

    FILE_SUFFIX = ".yaml"

    def __init__(
        self,
        root_dir: Optional[Union[str, Path]] = None,
        retired_root_dir: Optional[Union[str, Path]] = None,
    ):
        """Create the repository under ROS_HOME by default."""
        if root_dir is None:
            ros_home = Path(
                os.environ.get(
                    "ROS_HOME",
                    str(Path.home() / ".ros"),
                )
            )
            storage_root = ros_home / "fault_detector_spot"
            root_dir = storage_root / "sensors"
            if retired_root_dir is None:
                retired_root_dir = storage_root / "retired_sensors"
        self.root_dir = Path(root_dir).expanduser()
        if retired_root_dir is None:
            retired_root_dir = self.root_dir.parent / "retired_sensors"
        self.retired_root_dir = Path(retired_root_dir).expanduser()

    def get_sensor_path(self, sensor_id: str) -> Path:
        """Return the YAML path for a sensor ID."""
        validate_storage_name(sensor_id, "sensor ID")
        return self.root_dir / f"{sensor_id}{self.FILE_SUFFIX}"

    def exists(self, sensor_id: str) -> bool:
        """Return whether a physical sensor definition exists."""
        return self.get_sensor_path(sensor_id).is_file()

    def get_retired_sensor_path(self, sensor_id: str) -> Path:
        """Return a legacy retired-sensor path."""
        validate_storage_name(sensor_id, "sensor ID")
        return self.retired_root_dir / f"{sensor_id}{self.FILE_SUFFIX}"

    def is_retired(self, sensor_id: str) -> bool:
        """Return whether a legacy retired-sensor file exists."""
        return self.get_retired_sensor_path(sensor_id).is_file()

    def load(self, sensor_id: str) -> SensorDefinition:
        """Load and validate one physical sensor definition."""
        path = self.get_sensor_path(sensor_id)
        if not path.is_file():
            raise FileNotFoundError(
                f"Sensor definition does not exist: {path}"
            )
        try:
            with path.open("r", encoding="utf-8") as sensor_file:
                data = yaml.safe_load(sensor_file)
        except yaml.YAMLError as exception:
            raise ValueError(
                f"Invalid sensor YAML in {path}: {exception}"
            ) from exception
        definition = SensorDefinition.from_dict(data)
        if definition.sensor_id != sensor_id:
            raise ValueError(
                f"Sensor ID mismatch in {path}: {definition.sensor_id}"
            )
        definition.validate()
        return definition

    def create(self, definition: SensorDefinition) -> SensorDefinition:
        """Create a physical sensor definition without overwrite."""
        definition.validate()
        path = self.get_sensor_path(definition.sensor_id)
        if path.exists():
            raise FileExistsError(
                f"Sensor definition already exists: {definition.sensor_id}"
            )
        legacy_retired = self.get_retired_sensor_path(
            definition.sensor_id
        )
        if legacy_retired.exists():
            legacy_retired.unlink()
        self._write(definition)
        return definition

    def update(self, definition: SensorDefinition) -> SensorDefinition:
        """Replace an existing physical sensor definition atomically."""
        definition.validate()
        path = self.get_sensor_path(definition.sensor_id)
        if not path.is_file():
            raise FileNotFoundError(
                "Sensor definition does not exist: "
                f"{definition.sensor_id}"
            )
        self._write(definition)
        return definition

    def delete(self, sensor_id: str) -> SensorDefinition:
        """Delete one physical sensor definition and allow ID reuse."""
        definition = self.load(sensor_id)
        self.get_sensor_path(sensor_id).unlink()
        legacy_retired = self.get_retired_sensor_path(sensor_id)
        if legacy_retired.exists():
            legacy_retired.unlink()
        return definition

    def retire(self, sensor_id: str) -> SensorDefinition:
        """Delete one sensor through the existing retirement transport."""
        return self.delete(sensor_id)

    def list_sensor_ids(self) -> List[str]:
        """List stored physical sensor IDs in stable order."""
        if not self.root_dir.is_dir():
            return []
        return sorted(
            path.stem
            for path in self.root_dir.glob(f"*{self.FILE_SUFFIX}")
            if path.is_file()
        )

    def load_all(self) -> List[SensorDefinition]:
        """Load every available physical sensor definition."""
        return [
            self.load(sensor_id)
            for sensor_id in self.list_sensor_ids()
        ]

    def list_retired_sensor_ids(self) -> List[str]:
        """List legacy retired-sensor files still on disk."""
        if not self.retired_root_dir.is_dir():
            return []
        return sorted(
            path.stem
            for path in self.retired_root_dir.glob(
                f"*{self.FILE_SUFFIX}"
            )
            if path.is_file()
        )

    def _write(self, definition: SensorDefinition) -> None:
        content = yaml.safe_dump(
            definition.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(
            self.get_sensor_path(definition.sensor_id),
            content,
        )
