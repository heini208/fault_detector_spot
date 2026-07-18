"""Repository for portable inspection objects."""

import os
import shutil
from copy import deepcopy
from pathlib import Path
from typing import List, Optional, Union

import yaml

from .models import ObjectDefinition
from .repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


class ObjectRepository:
    """Load and save map-independent object definitions."""

    FILE_NAME = "object.yaml"

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
                / "objects"
            )

        self.root_dir = Path(root_dir).expanduser()

    def get_object_dir(self, object_id: str) -> Path:
        """Return the directory for an object."""
        validate_storage_name(object_id, "object ID")
        return self.root_dir / object_id

    def get_object_path(self, object_id: str) -> Path:
        """Return the YAML path for an object."""
        return self.get_object_dir(object_id) / self.FILE_NAME

    def exists(self, object_id: str) -> bool:
        """Return whether an object definition exists."""
        return self.get_object_path(object_id).is_file()

    def load(
        self,
        object_id: str,
        validate: bool = True,
    ) -> ObjectDefinition:
        """Load an object definition."""
        path = self.get_object_path(object_id)

        if not path.is_file():
            raise FileNotFoundError(
                f"Object definition does not exist: {path}"
            )

        try:
            with path.open("r", encoding="utf-8") as object_file:
                data = yaml.safe_load(object_file)
        except yaml.YAMLError as exception:
            raise ValueError(
                f"Invalid object YAML in {path}: {exception}"
            ) from exception

        if not isinstance(data, dict):
            raise ValueError(
                f"Object root must be an object: {path}"
            )

        definition = ObjectDefinition.from_dict(data)

        if definition.object_id != object_id:
            raise ValueError(
                f"Object ID mismatch in {path}: "
                f"{definition.object_id}"
            )

        if validate:
            definition.validate()

        return definition

    def save(
        self,
        definition: ObjectDefinition,
        validate: bool = True,
    ) -> Path:
        """Validate and atomically save an object."""
        validate_storage_name(definition.object_id, "object ID")

        current = deepcopy(definition)

        if validate:
            current.validate()

        path = self.get_object_path(current.object_id)
        content = yaml.safe_dump(
            current.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(path, content)
        return path

    def list_object_ids(self) -> List[str]:
        """List stored object IDs."""
        if not self.root_dir.is_dir():
            return []

        return sorted(
            directory.name
            for directory in self.root_dir.iterdir()
            if directory.is_dir()
            and (directory / self.FILE_NAME).is_file()
        )

    def delete(self, object_id: str) -> bool:
        """Delete an object directory."""
        object_dir = self.get_object_dir(object_id)

        if not object_dir.exists():
            return False

        shutil.rmtree(object_dir)
        return True