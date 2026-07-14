"""Repository for reusable inspection definitions."""

import os
import shutil
from pathlib import Path
from typing import List, Optional, Union

import yaml

from fault_detector_spot.inspection.models import (
    InspectionDefinition,
)
from fault_detector_spot.inspection.repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


class InspectionRepository:
    """Load and save reusable inspection configurations."""

    FILE_NAME = "inspection.yaml"

    def __init__(
        self,
        root_dir: Optional[Union[str, Path]] = None,
    ):
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

    def get_map_dir(self, map_name: str) -> Path:
        """Return the inspection directory for a map."""
        validate_storage_name(map_name, "map name")
        return self.root_dir / map_name

    def get_inspection_dir(
        self,
        map_name: str,
        inspection_id: str,
    ) -> Path:
        """Return the directory for an inspection."""
        validate_storage_name(inspection_id, "inspection ID")
        return self.get_map_dir(map_name) / inspection_id

    def get_inspection_path(
        self,
        map_name: str,
        inspection_id: str,
    ) -> Path:
        """Return the YAML path for an inspection."""
        return (
            self.get_inspection_dir(
                map_name,
                inspection_id,
            )
            / self.FILE_NAME
        )

    def exists(
        self,
        map_name: str,
        inspection_id: str,
    ) -> bool:
        """Return whether an inspection exists."""
        return self.get_inspection_path(
            map_name,
            inspection_id,
        ).is_file()

    def load(
        self,
        map_name: str,
        inspection_id: str,
        validate: bool = True,
    ) -> InspectionDefinition:
        """Load an inspection definition."""
        path = self.get_inspection_path(
            map_name,
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

        if inspection.map_name != map_name:
            raise ValueError(
                f"Inspection map mismatch in {path}: "
                f"{inspection.map_name}"
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
        """Validate and atomically save an inspection."""
        validate_storage_name(
            inspection.map_name,
            "map name",
        )
        validate_storage_name(
            inspection.inspection_id,
            "inspection ID",
        )

        if validate:
            inspection.validate()

        path = self.get_inspection_path(
            inspection.map_name,
            inspection.inspection_id,
        )

        content = yaml.safe_dump(
            inspection.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )

        atomic_write_text(path, content)
        return path

    def list_inspection_ids(
        self,
        map_name: str,
    ) -> List[str]:
        """List saved inspections for a map."""
        map_dir = self.get_map_dir(map_name)

        if not map_dir.is_dir():
            return []

        return sorted(
            directory.name
            for directory in map_dir.iterdir()
            if directory.is_dir()
            and (directory / self.FILE_NAME).is_file()
        )

    def delete(
        self,
        map_name: str,
        inspection_id: str,
    ) -> bool:
        """Delete an inspection directory."""
        inspection_dir = self.get_inspection_dir(
            map_name,
            inspection_id,
        )

        if not inspection_dir.exists():
            return False

        shutil.rmtree(inspection_dir)
        return True