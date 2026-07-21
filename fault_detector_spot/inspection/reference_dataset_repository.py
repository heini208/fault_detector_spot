"""Atomic storage for reference-view sensor datasets."""

import json
import os
import shutil
import tempfile
from dataclasses import replace
from pathlib import Path
from typing import Optional, Tuple, Union

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.models import ReferenceView
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.repository_utils import (
    atomic_write_text,
    validate_storage_name,
)


ReferenceDataset = Tuple[Image, Image, CameraInfo]


class ReferenceDatasetRepository:
    """Persist immutable reference-view datasets under an object."""

    def __init__(
        self,
        object_root: Optional[Union[str, Path]] = None,
    ):
        """Use the same root as the object repository."""
        self.object_repository = ObjectRepository(object_root)

    def save(
        self,
        object_id: str,
        routine_id: str,
        reference_view: ReferenceView,
        rgb_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> ReferenceView:
        """Atomically publish one immutable reference dataset."""
        _validate_request(object_id, routine_id, reference_view)
        if reference_view.reference_dataset_path is not None:
            raise ValueError(
                "Reference view already has a dataset path"
            )

        dataset_id = _capture_id(rgb_image)
        relative_path = (
            Path("reference_datasets") / routine_id / dataset_id
        )
        stored_view = replace(
            reference_view,
            reference_dataset_path=relative_path.as_posix(),
        )
        dataset_path = (
            self.object_repository.get_object_dir(object_id)
            / relative_path
        )
        if dataset_path.exists():
            raise FileExistsError(
                f"Reference dataset already exists: {dataset_path}"
            )

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
        try:
            for name, content in files.items():
                _write_bytes(staging_path / name, content)
            atomic_write_text(
                staging_path / "metadata.json",
                metadata,
            )
            os.replace(staging_path, dataset_path)
        except Exception:
            shutil.rmtree(staging_path, ignore_errors=True)
            raise

        return stored_view

    def load(
        self,
        object_id: str,
        routine_id: str,
        reference_view: ReferenceView,
    ) -> ReferenceDataset:
        """Reload the ROS messages referenced by a saved view."""
        _validate_request(object_id, routine_id, reference_view)
        dataset_path = self._dataset_path(
            object_id,
            routine_id,
            reference_view,
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

    def _dataset_path(
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
            != ("reference_datasets", routine_id)
        ):
            raise ValueError(
                "Reference dataset path is outside the selected routine"
            )
        return (
            self.object_repository.get_object_dir(object_id)
            / relative_path
        )


def _validate_request(
    object_id: str,
    routine_id: str,
    reference_view: ReferenceView,
) -> None:
    validate_storage_name(object_id, "object ID")
    validate_storage_name(routine_id, "routine ID")
    reference_view.validate()


def _capture_id(rgb_image: Image) -> str:
    stamp = rgb_image.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("RGB image timestamp is invalid")
    if stamp.sec == 0 and stamp.nanosec == 0:
        raise ValueError("RGB image timestamp must not be zero")
    return f"{stamp.sec}_{stamp.nanosec:09d}"


def _write_bytes(path: Path, content: bytes) -> None:
    path.write_bytes(content)
