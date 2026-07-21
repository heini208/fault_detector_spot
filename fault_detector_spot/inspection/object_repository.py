"""Repository for portable inspection objects and their datasets."""

import json
import os
import shutil
import tempfile
from dataclasses import replace
from pathlib import Path
from typing import List, Optional, Tuple, Union

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image

from .models import InspectionObject, ReferenceView
from .repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


ReferenceDataset = Tuple[Image, Image, CameraInfo]


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

    def save_reference_dataset(
        self,
        object_id: str,
        routine_id: str,
        reference_view: ReferenceView,
        rgb_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> InspectionObject:
        """Save a routine dataset and update its object definition."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        reference_view.validate()
        if reference_view.reference_dataset_path is not None:
            raise ValueError(
                "New reference view already has a dataset path"
            )

        definition = self.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")

        old_dataset_path = None
        if routine.reference_view.reference_dataset_path is not None:
            old_dataset_path = self._reference_dataset_path(
                object_id,
                routine_id,
                routine.reference_view,
            )

        dataset_id = _capture_id(rgb_image)
        relative_path = (
            Path(self.REFERENCE_DATASET_DIRECTORY)
            / routine_id
            / dataset_id
        )
        dataset_path = (
            self.get_object_dir(object_id)
            / relative_path
        )
        if dataset_path.exists():
            raise FileExistsError(
                f"Reference dataset already exists: {dataset_path}"
            )

        stored_view = replace(
            reference_view,
            reference_dataset_path=relative_path.as_posix(),
        )
        stored_routine = replace(
            routine,
            reference_view=stored_view,
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
        stored_definition.validate()

        files = {
            "rgb.cdr": bytes(serialize_message(rgb_image)),
            "depth.cdr": bytes(serialize_message(depth_image)),
            "camera_info.cdr": bytes(
                serialize_message(camera_info)
            ),
        }
        metadata = json.dumps(
            {
                "object_id": object_id,
                "routine_id": routine_id,
                "capture_timestamp": {
                    "sec": rgb_image.header.stamp.sec,
                    "nanosec": rgb_image.header.stamp.nanosec,
                },
            },
            indent=2,
        ) + "\n"

        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        staging_path = Path(tempfile.mkdtemp(
            prefix=f".{dataset_id}.",
            suffix=".tmp",
            dir=str(dataset_path.parent),
        ))
        published = False
        try:
            for name, content in files.items():
                _write_bytes(staging_path / name, content)
            atomic_write_text(
                staging_path / "metadata.json",
                metadata,
            )
            os.replace(staging_path, dataset_path)
            published = True
            self.save(stored_definition)
        except Exception:
            shutil.rmtree(staging_path, ignore_errors=True)
            if published:
                shutil.rmtree(dataset_path, ignore_errors=True)
            raise

        if (
            old_dataset_path is not None
            and old_dataset_path != dataset_path
        ):
            shutil.rmtree(old_dataset_path, ignore_errors=True)

        return stored_definition

    def load_reference_dataset(
        self,
        object_id: str,
        routine_id: str,
    ) -> ReferenceDataset:
        """Load the sensor dataset owned by an object's routine."""
        validate_storage_name(routine_id, "routine ID")
        definition = self.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")

        dataset_path = self._reference_dataset_path(
            object_id,
            routine_id,
            routine.reference_view,
        )
        metadata = json.loads(
            (dataset_path / "metadata.json").read_text(
                encoding="utf-8"
            )
        )
        if metadata.get("object_id") != object_id:
            raise ValueError(
                "Reference dataset object ID does not match"
            )
        if metadata.get("routine_id") != routine_id:
            raise ValueError(
                "Reference dataset routine ID does not match"
            )

        return (
            deserialize_message(
                (dataset_path / "rgb.cdr").read_bytes(),
                Image,
            ),
            deserialize_message(
                (dataset_path / "depth.cdr").read_bytes(),
                Image,
            ),
            deserialize_message(
                (dataset_path / "camera_info.cdr").read_bytes(),
                CameraInfo,
            ),
        )

    def _reference_dataset_path(
        self,
        object_id: str,
        routine_id: str,
        reference_view: ReferenceView,
    ) -> Path:
        value = reference_view.reference_dataset_path
        if value is None:
            raise ValueError(
                "Reference view does not have a dataset path"
            )
        relative_path = Path(value)
        if (
            relative_path.is_absolute()
            or len(relative_path.parts) != 3
            or relative_path.parts[:2]
            != (self.REFERENCE_DATASET_DIRECTORY, routine_id)
        ):
            raise ValueError(
                "Reference dataset path is outside the selected routine"
            )
        return self.get_object_dir(object_id) / relative_path

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


def _capture_id(rgb_image: Image) -> str:
    stamp = rgb_image.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("RGB image timestamp is invalid")
    if stamp.sec == 0 and stamp.nanosec == 0:
        raise ValueError("RGB image timestamp must not be zero")
    return f"{stamp.sec}_{stamp.nanosec:09d}"


def _write_bytes(path: Path, content: bytes) -> None:
    path.write_bytes(content)
