"""Repository for object-scoped inspection definitions."""

import os
import shutil
from copy import deepcopy
from pathlib import Path
from typing import List, Optional, Union

import yaml

from .models import InspectionDefinition
from .repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


class InspectionRepository:
    """Load current inspections with optional legacy fallback."""

    FILE_NAME = "inspection.yaml"

    def __init__(
        self,
        root_dir: Optional[Union[str, Path]] = None,
    ):
        """Create a repository under ROS_HOME by default."""
        if root_dir is None:
            ros_home = Path(
                os.environ.get(
                    "ROS_HOME",
                    str(Path.home() / ".ros"),
                )
            )
            root_dir = (
                ros_home
                / "fault_detector_spot"
                / "inspections"
            )

        self.root_dir = Path(root_dir).expanduser()

    def get_object_dir(self, object_id: str) -> Path:
        """Return the canonical inspection directory for an object."""
        validate_storage_name(object_id, "object ID")
        return self.root_dir / object_id

    def get_inspection_dir(
        self,
        object_id: str,
        inspection_id: str,
    ) -> Path:
        """Return the canonical directory for an inspection."""
        validate_storage_name(inspection_id, "inspection ID")
        return self.get_object_dir(object_id) / inspection_id

    def get_inspection_path(
        self,
        object_id: str,
        inspection_id: str,
    ) -> Path:
        """Return the canonical YAML path for an inspection."""
        return (
            self.get_inspection_dir(object_id, inspection_id)
            / self.FILE_NAME
        )

    def get_legacy_inspection_path(
        self,
        map_name: str,
        inspection_id: str,
    ) -> Path:
        """Return an old map-scoped inspection path."""
        validate_storage_name(map_name, "map name")
        validate_storage_name(inspection_id, "inspection ID")
        return self.root_dir / map_name / inspection_id / self.FILE_NAME

    def exists(
        self,
        object_id: str,
        inspection_id: str,
    ) -> bool:
        """Return whether a canonical inspection exists."""
        return self.get_inspection_path(
            object_id,
            inspection_id,
        ).is_file()

    def load(
        self,
        object_id: str,
        inspection_id: str,
        validate: bool = True,
        legacy_map_name: Optional[str] = None,
    ) -> InspectionDefinition:
        """Load canonical data or explicitly requested legacy data."""
        path = self.get_inspection_path(
            object_id,
            inspection_id,
        )

        if not path.is_file() and legacy_map_name:
            path = self.get_legacy_inspection_path(
                legacy_map_name,
                inspection_id,
            )

        if not path.is_file():
            raise FileNotFoundError(
                f"Inspection does not exist: {path}"
            )

        try:
            with path.open(
                "r",
                encoding="utf-8",
            ) as inspection_file:
                data = yaml.safe_load(inspection_file)
        except yaml.YAMLError as exception:
            raise ValueError(
                f"Invalid inspection YAML in {path}: {exception}"
            ) from exception

        if not isinstance(data, dict):
            raise ValueError(
                f"Inspection root must be an object: {path}"
            )

        inspection = InspectionDefinition.from_dict(data)

        if inspection.object_id != object_id:
            raise ValueError(
                f"Inspection object mismatch in {path}: "
                f"{inspection.object_id}"
            )

        if inspection.inspection_id != inspection_id:
            raise ValueError(
                f"Inspection ID mismatch in {path}: "
                f"{inspection.inspection_id}"
            )

        if validate:
            inspection.validate()

        return inspection

    def save(
        self,
        inspection: InspectionDefinition,
        validate: bool = True,
    ) -> Path:
        validate_storage_name(inspection.object_id, "object ID")
        validate_storage_name(
            inspection.inspection_id,
            "inspection ID",
        )

        current = deepcopy(inspection)

        if not current.preferred_execution_frame:
            current.preferred_execution_frame = "odom"

        if validate:
            current.validate()

        path = self.get_inspection_path(
            current.object_id,
            current.inspection_id,
        )
        content = yaml.safe_dump(
            current.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(path, content)
        return path

    def list_inspection_ids(
        self,
        object_id: str,
    ) -> List[str]:
        """List canonical inspections for an object."""
        object_dir = self.get_object_dir(object_id)

        if not object_dir.is_dir():
            return []

        return sorted(
            directory.name
            for directory in object_dir.iterdir()
            if directory.is_dir()
            and (directory / self.FILE_NAME).is_file()
        )

    def delete(
        self,
        object_id: str,
        inspection_id: str,
    ) -> bool:
        """Delete only the canonical inspection directory."""
        inspection_dir = self.get_inspection_dir(
            object_id,
            inspection_id,
        )

        if not inspection_dir.exists():
            return False

        shutil.rmtree(inspection_dir)
        return True