"""Repository for portable inspection object definitions."""

import os
import shutil
from dataclasses import replace
from pathlib import Path
from typing import List, Optional, Union

import yaml

from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ProbePoint,
)
from fault_detector_spot.shared.persistence.file_storage import (
    atomic_write_text,
    validate_storage_name,
)


class ObjectRepository:
    """Load and save complete map-independent inspection objects."""

    FILE_NAME = "object.yaml"
    REFERENCE_DATASET_DIRECTORY = "reference_datasets"

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
    ) -> InspectionObject:
        """Load a complete inspection object."""
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

        definition = InspectionObject.from_dict(data)

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
        definition: InspectionObject,
        validate: bool = True,
    ) -> Path:
        """Validate and atomically save an object."""
        validate_storage_name(definition.object_id, "object ID")

        if validate:
            definition.validate()

        path = self.get_object_path(definition.object_id)
        content = yaml.safe_dump(
            definition.to_dict(),
            sort_keys=False,
            allow_unicode=True,
        )
        atomic_write_text(path, content)
        return path

    def create(self, definition: InspectionObject) -> InspectionObject:
        """Create a new object without replacing an existing one."""
        validate_storage_name(definition.object_id, "object ID")
        definition.validate()
        if self.exists(definition.object_id):
            raise FileExistsError(
                "Inspection object already exists: "
                f"{definition.object_id}"
            )
        self.save(definition, validate=False)
        return definition

    def add_routine(
        self,
        object_id: str,
        routine: InspectionRoutine,
    ) -> InspectionObject:
        """Add a new routine to an existing inspection object."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine.routine_id, "routine ID")
        routine.validate()

        definition = self.load(object_id)
        if definition.get_routine(routine.routine_id) is not None:
            raise FileExistsError(
                "Inspection routine already exists: "
                f"{object_id}/{routine.routine_id}"
            )

        stored_definition = replace(
            definition,
            routines=[*definition.routines, routine],
        )
        self.save(stored_definition)
        return stored_definition

    def add_probe_point(
        self,
        object_id: str,
        routine_id: str,
        probe_point: ProbePoint,
    ) -> InspectionObject:
        """Append one new probe point to an existing routine."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        probe_point.validate()

        definition = self.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(
                "Inspection routine does not exist: "
                f"{object_id}/{routine_id}"
            )
        if routine.get_probe_point(probe_point.probe_point_id) is not None:
            raise FileExistsError(
                "Probe point already exists: "
                f"{object_id}/{routine_id}/"
                f"{probe_point.probe_point_id}"
            )

        stored_routine = replace(
            routine,
            probe_points=[*routine.probe_points, probe_point],
        )
        stored_definition = replace(
            definition,
            routines=[
                stored_routine
                if candidate.routine_id == routine_id
                else candidate
                for candidate in definition.routines
            ],
        )
        self.save(stored_definition)
        return stored_definition

    def replace_probe_point(
        self,
        object_id: str,
        routine_id: str,
        probe_point: ProbePoint,
    ) -> InspectionObject:
        """Atomically replace one existing probe point in place."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        probe_point.validate()

        definition = self.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(
                "Inspection routine does not exist: "
                f"{object_id}/{routine_id}"
            )
        if routine.get_probe_point(probe_point.probe_point_id) is None:
            raise KeyError(
                "Probe point does not exist: "
                f"{object_id}/{routine_id}/"
                f"{probe_point.probe_point_id}"
            )

        stored_routine = replace(
            routine,
            probe_points=[
                probe_point
                if candidate.probe_point_id
                == probe_point.probe_point_id
                else candidate
                for candidate in routine.probe_points
            ],
        )
        stored_definition = replace(
            definition,
            routines=[
                stored_routine
                if candidate.routine_id == routine_id
                else candidate
                for candidate in definition.routines
            ],
        )
        self.save(stored_definition)
        return stored_definition

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

    def find_sensor_references(self, sensor_id: str) -> List[str]:
        """List saved object and routine paths using a sensor ID."""
        validate_storage_name(sensor_id, "sensor ID")
        references = []
        for object_id in self.list_object_ids():
            path = self.get_object_path(object_id)
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
            routines = data.get("routines")
            if not isinstance(routines, list):
                raise ValueError(
                    f"Object routines must be a list: {path}"
                )
            for index, routine in enumerate(routines):
                if not isinstance(routine, dict):
                    raise ValueError(
                        "Inspection routine must be an object: "
                        f"{path} at index {index}"
                    )
                stored_sensor_id = routine.get("sensor_id")
                if not isinstance(stored_sensor_id, str):
                    raise ValueError(
                        "Inspection routine requires a sensor ID: "
                        f"{path} at index {index}"
                    )
                if stored_sensor_id != sensor_id:
                    continue
                routine_id = routine.get("routine_id")
                if not isinstance(routine_id, str) or not routine_id.strip():
                    raise ValueError(
                        "Referenced inspection routine requires an ID: "
                        f"{path} at index {index}"
                    )
                references.append(f"{object_id}/{routine_id}")
        return references

    def delete_object(self, object_id: str) -> bool:
        """Delete an object and every dataset owned by it."""
        object_dir = self.get_object_dir(object_id)

        if not object_dir.exists():
            return False

        shutil.rmtree(object_dir)
        return True

    def delete_routine(
        self,
        object_id: str,
        routine_id: str,
    ) -> InspectionObject:
        """Delete a routine and every dataset owned by it."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        definition = self.load(object_id)
        if definition.get_routine(routine_id) is None:
            raise KeyError(
                "Inspection routine does not exist: "
                f"{object_id}/{routine_id}"
            )

        stored_definition = replace(
            definition,
            routines=[
                routine
                for routine in definition.routines
                if routine.routine_id != routine_id
            ],
        )
        self.save(stored_definition)
        routine_dataset_dir = (
            self.get_object_dir(object_id)
            / self.REFERENCE_DATASET_DIRECTORY
            / routine_id
        )
        if routine_dataset_dir.exists():
            shutil.rmtree(routine_dataset_dir)
        return stored_definition

    def delete(self, object_id: str) -> bool:
        """Delete an object through the retained repository API."""
        return self.delete_object(object_id)
