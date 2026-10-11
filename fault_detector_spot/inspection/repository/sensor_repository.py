"""Repository for hand-mounted sensor transform definitions."""

import os
from pathlib import Path
from typing import List, Optional, Union

import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    DEFAULT_HAND_SENSOR_ID,
    SensorDefinition,
)
from fault_detector_spot.inspection.model.sensor_geometry import SensorHeadGeometry
from fault_detector_spot.inspection.repository.sensor_mesh_store import SensorMeshStore
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
        self.root_dir = Path(root_dir).expanduser()
        self.mesh_store = SensorMeshStore(self.root_dir / "assets")

    def store_collision_chunk(
        self, sha256: str, byte_count: int, offset: int, data: bytes,
    ) -> bool:
        """Upload collision bytes into managed storage before saving a definition."""
        return self.mesh_store.store_chunk(sha256, byte_count, offset, data)

    def read_collision_mesh(self, geometry: SensorHeadGeometry) -> bytes:
        """Read a validated asset without relying on the original import file."""
        return self.mesh_store.read(geometry)

    def read_collision_chunk(
        self, sha256: str, byte_count: int, offset: int,
    ) -> bytes:
        """Return a bounded mesh chunk for another application process."""
        return self.mesh_store.read_chunk(sha256, byte_count, offset)

    def get_sensor_path(self, sensor_id: str) -> Path:
        """Return the YAML path for a sensor ID."""
        validate_storage_name(sensor_id, "sensor ID")
        return self.root_dir / f"{sensor_id}{self.FILE_SUFFIX}"

    def exists(self, sensor_id: str) -> bool:
        """Return whether a physical sensor definition exists."""
        return self.get_sensor_path(sensor_id).is_file()

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
        self._validate_geometry_asset(definition)
        return definition

    def create(self, definition: SensorDefinition) -> SensorDefinition:
        """Create a physical sensor definition without overwrite."""
        self._require_editable(definition.sensor_id)
        definition.validate()
        path = self.get_sensor_path(definition.sensor_id)
        if path.exists():
            raise FileExistsError(
                f"Sensor definition already exists: {definition.sensor_id}"
            )
        self._write(definition)
        return definition

    def update(self, definition: SensorDefinition) -> SensorDefinition:
        """Replace an existing physical sensor definition atomically."""
        self._require_editable(definition.sensor_id)
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
        self._require_editable(sensor_id)
        definition = self.load(sensor_id)
        self.get_sensor_path(sensor_id).unlink()
        return definition

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

    @staticmethod
    def _require_editable(sensor_id: str) -> None:
        if sensor_id == DEFAULT_HAND_SENSOR_ID:
            raise ValueError("The default hand configuration is read-only")

    def _write(self, definition: SensorDefinition) -> None:
        self._validate_geometry_asset(definition)
        content = yaml.safe_dump(
            definition.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(
            self.get_sensor_path(definition.sensor_id),
            content,
        )

    def _validate_geometry_asset(self, definition: SensorDefinition) -> None:
        if definition.head_geometry is not None:
            self.read_collision_mesh(definition.head_geometry)
