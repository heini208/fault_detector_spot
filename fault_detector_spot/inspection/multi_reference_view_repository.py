"""Persistence for up to three camera-specific routine reference views."""

import json
import os
import shutil
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image

from .models import InspectionObject, ReferenceView
from .object_repository import ObjectRepository
from .repository_utils import atomic_write_text, validate_storage_name


@dataclass(frozen=True)
class CapturedReferenceView:
    """One camera-specific synchronized reference dataset."""

    slot_index: int
    camera_id: str
    reference_view: ReferenceView
    rgb_image: Image
    depth_image: Image
    camera_info: CameraInfo


class MultiReferenceViewRepository:
    """Store a primary model view plus camera-specific dataset metadata."""

    INDEX_FILE_NAME = "reference_views.yaml"

    def __init__(
        self,
        root_dir: Optional[Union[str, Path]] = None,
    ):
        self.object_repository = ObjectRepository(root_dir)

    @property
    def root_dir(self) -> Path:
        return self.object_repository.root_dir

    def has_reference_views(self, object_id: str, routine_id: str) -> bool:
        """Return whether a routine owns any captured reference view."""
        definition = self.object_repository.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")
        return routine.reference_view is not None

    def save_reference_views(
        self,
        object_id: str,
        routine_id: str,
        captures: Sequence[CapturedReferenceView],
    ) -> InspectionObject:
        """Atomically replace one routine's complete camera-view set."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        normalized = self._validate_captures(captures)

        definition = self.object_repository.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")

        routine_root = (
            self.object_repository.get_object_dir(object_id)
            / self.object_repository.REFERENCE_DATASET_DIRECTORY
            / routine_id
        )
        routine_root.mkdir(parents=True, exist_ok=True)
        staging_root = Path(tempfile.mkdtemp(
            prefix=".reference_views.",
            suffix=".tmp",
            dir=str(routine_root),
        ))

        stored_entries = []
        published_paths = []
        index_path = routine_root / self.INDEX_FILE_NAME
        previous_index = (
            index_path.read_text(encoding="utf-8")
            if index_path.is_file()
            else None
        )
        object_was_saved = False
        try:
            for capture in normalized:
                dataset_id = self._dataset_id(capture)
                final_path = routine_root / dataset_id
                if final_path.exists():
                    raise FileExistsError(
                        f"Reference dataset already exists: {final_path}"
                    )
                staged_path = staging_root / dataset_id
                staged_path.mkdir(parents=True)
                self._write_dataset(staged_path, object_id, routine_id, capture)
                relative_path = (
                    Path(self.object_repository.REFERENCE_DATASET_DIRECTORY)
                    / routine_id
                    / dataset_id
                )
                stored_view = replace(
                    capture.reference_view,
                    reference_dataset_path=relative_path.as_posix(),
                )
                stored_entries.append({
                    "slot_index": capture.slot_index,
                    "camera_id": capture.camera_id,
                    "reference_view": stored_view.to_dict(),
                })

            primary_view = ReferenceView.from_dict(
                stored_entries[0]["reference_view"]
            )
            stored_routine = replace(routine, reference_view=primary_view)
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

            for entry in stored_entries:
                dataset_id = Path(
                    entry["reference_view"]["reference_dataset_path"]
                ).name
                source = staging_root / dataset_id
                destination = routine_root / dataset_id
                os.replace(source, destination)
                published_paths.append(destination)

            index_content = yaml.safe_dump(
                {"reference_views": stored_entries},
                sort_keys=False,
                allow_unicode=True,
            )
            atomic_write_text(index_path, index_content)
            self.object_repository.save(stored_definition)
            object_was_saved = True
        except Exception:
            for path in published_paths:
                shutil.rmtree(path, ignore_errors=True)
            if previous_index is None:
                index_path.unlink(missing_ok=True)
            else:
                atomic_write_text(index_path, previous_index)
            if object_was_saved:
                self.object_repository.save(definition)
            raise
        finally:
            shutil.rmtree(staging_root, ignore_errors=True)

        self._delete_unreferenced_datasets(routine_root, stored_entries)
        return stored_definition

    def load_reference_views(
        self,
        object_id: str,
        routine_id: str,
    ) -> List[CapturedReferenceView]:
        """Load all camera-specific datasets ordered by display slot."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        definition = self.object_repository.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")
        if routine.reference_view is None:
            return []

        routine_root = (
            self.object_repository.get_object_dir(object_id)
            / self.object_repository.REFERENCE_DATASET_DIRECTORY
            / routine_id
        )
        index_path = routine_root / self.INDEX_FILE_NAME
        if not index_path.is_file():
            rgb, depth, camera_info = (
                self.object_repository.load_reference_dataset(
                    object_id,
                    routine_id,
                )
            )
            return [CapturedReferenceView(
                slot_index=0,
                camera_id="hand",
                reference_view=routine.reference_view,
                rgb_image=rgb,
                depth_image=depth,
                camera_info=camera_info,
            )]

        data = yaml.safe_load(index_path.read_text(encoding="utf-8"))
        entries = data.get("reference_views") if isinstance(data, dict) else None
        if not isinstance(entries, list):
            raise ValueError(f"Invalid reference-view index: {index_path}")

        result = []
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f"Invalid reference-view entry: {index_path}")
            slot_index = int(entry["slot_index"])
            camera_id = str(entry["camera_id"])
            reference_view = ReferenceView.from_dict(entry["reference_view"])
            dataset_path = self._dataset_path(
                object_id,
                routine_id,
                reference_view,
            )
            metadata = json.loads(
                (dataset_path / "metadata.json").read_text(encoding="utf-8")
            )
            if metadata.get("object_id") != object_id:
                raise ValueError("Reference dataset object ID does not match")
            if metadata.get("routine_id") != routine_id:
                raise ValueError("Reference dataset routine ID does not match")
            if metadata.get("camera_id") != camera_id:
                raise ValueError("Reference dataset camera ID does not match")
            if int(metadata.get("slot_index", -1)) != slot_index:
                raise ValueError("Reference dataset slot index does not match")
            result.append(CapturedReferenceView(
                slot_index=slot_index,
                camera_id=camera_id,
                reference_view=reference_view,
                rgb_image=deserialize_message(
                    (dataset_path / "rgb.cdr").read_bytes(),
                    Image,
                ),
                depth_image=deserialize_message(
                    (dataset_path / "depth.cdr").read_bytes(),
                    Image,
                ),
                camera_info=deserialize_message(
                    (dataset_path / "camera_info.cdr").read_bytes(),
                    CameraInfo,
                ),
            ))
        return sorted(result, key=lambda capture: capture.slot_index)

    @staticmethod
    def _validate_captures(
        captures: Sequence[CapturedReferenceView],
    ) -> Tuple[CapturedReferenceView, ...]:
        normalized = tuple(sorted(captures, key=lambda item: item.slot_index))
        if not 1 <= len(normalized) <= 3:
            raise ValueError("Reference capture must contain one to three views")
        slots = [capture.slot_index for capture in normalized]
        cameras = [capture.camera_id for capture in normalized]
        if any(slot < 0 or slot > 2 for slot in slots):
            raise ValueError("Reference camera slot must be between 0 and 2")
        if len(set(slots)) != len(slots):
            raise ValueError("Reference camera slots must be unique")
        if len(set(cameras)) != len(cameras):
            raise ValueError("Reference camera IDs must be unique")
        for capture in normalized:
            if not capture.camera_id.strip():
                raise ValueError("Reference camera ID must not be empty")
            capture.reference_view.validate()
            if capture.reference_view.reference_dataset_path is not None:
                raise ValueError(
                    "New reference view already has a dataset path"
                )
        return normalized

    def _dataset_path(
        self,
        object_id: str,
        routine_id: str,
        reference_view: ReferenceView,
    ) -> Path:
        value = reference_view.reference_dataset_path
        if value is None:
            raise ValueError("Reference view does not have a dataset path")
        relative = Path(value)
        if (
            relative.is_absolute()
            or len(relative.parts) != 3
            or relative.parts[:2]
            != (
                self.object_repository.REFERENCE_DATASET_DIRECTORY,
                routine_id,
            )
        ):
            raise ValueError(
                "Reference dataset path is outside the selected routine"
            )
        return self.object_repository.get_object_dir(object_id) / relative

    @staticmethod
    def _dataset_id(capture: CapturedReferenceView) -> str:
        stamp = capture.rgb_image.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("RGB image timestamp is invalid")
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise ValueError("RGB image timestamp must not be zero")
        return (
            f"slot{capture.slot_index + 1}_{capture.camera_id}_"
            f"{stamp.sec}_{stamp.nanosec:09d}"
        )

    @staticmethod
    def _write_dataset(
        path: Path,
        object_id: str,
        routine_id: str,
        capture: CapturedReferenceView,
    ) -> None:
        files = {
            "rgb.cdr": bytes(serialize_message(capture.rgb_image)),
            "depth.cdr": bytes(serialize_message(capture.depth_image)),
            "camera_info.cdr": bytes(serialize_message(capture.camera_info)),
        }
        for name, content in files.items():
            file_path = path / name
            with file_path.open("wb") as output:
                output.write(content)
                output.flush()
                os.fsync(output.fileno())
        metadata = {
            "object_id": object_id,
            "routine_id": routine_id,
            "camera_id": capture.camera_id,
            "slot_index": capture.slot_index,
            "capture_timestamp": {
                "sec": capture.rgb_image.header.stamp.sec,
                "nanosec": capture.rgb_image.header.stamp.nanosec,
            },
        }
        atomic_write_text(
            path / "metadata.json",
            json.dumps(metadata, indent=2) + "\n",
        )

    def _delete_unreferenced_datasets(self, routine_root, entries) -> None:
        referenced = {
            Path(entry["reference_view"]["reference_dataset_path"]).name
            for entry in entries
        }
        for child in routine_root.iterdir():
            if child.is_dir() and child.name not in referenced:
                shutil.rmtree(child, ignore_errors=True)
