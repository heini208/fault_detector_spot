"""Repository for immutable hand-mounted sensor calibrations."""

import os
from pathlib import Path
from typing import List, Optional, Union

import yaml

from .repository_utils import atomic_write_text, validate_storage_name
from .sensor_models import SensorDefinition


class SensorRepository:
    """Load and create one YAML definition per mounted sensor."""

    FILE_SUFFIX = ".yaml"

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
            root_dir = ros_home / "fault_detector_spot" / "sensors"
        self.root_dir = Path(root_dir).expanduser()

    def get_sensor_path(self, sensor_id: str) -> Path:
        """Return the YAML path for a sensor ID."""
        validate_storage_name(sensor_id, "sensor ID")
        return self.root_dir / f"{sensor_id}{self.FILE_SUFFIX}"

    def exists(self, sensor_id: str) -> bool:
        """Return whether a definition exists."""
        return self.get_sensor_path(sensor_id).is_file()

    def load(self, sensor_id: str) -> SensorDefinition:
        """Load and validate one sensor definition."""
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
        """Create a sensor definition without allowing overwrite."""
        definition.validate()
        path = self.get_sensor_path(definition.sensor_id)
        if path.exists():
            raise FileExistsError(
                f"Sensor definition already exists: {definition.sensor_id}"
            )
        content = yaml.safe_dump(
            definition.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(path, content)
        return definition

    def list_sensor_ids(self) -> List[str]:
        """List stored sensor IDs in stable order."""
        if not self.root_dir.is_dir():
            return []
        return sorted(
            path.stem
            for path in self.root_dir.glob(f"*{self.FILE_SUFFIX}")
            if path.is_file()
        )

    def load_all(self) -> List[SensorDefinition]:
        """Load every stored sensor definition."""
        return [
            self.load(sensor_id)
            for sensor_id in self.list_sensor_ids()
        ]
