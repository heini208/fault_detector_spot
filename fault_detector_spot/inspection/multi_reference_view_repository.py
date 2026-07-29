"""Atomic persistence for camera-specific routine reference-view sets."""

import json
import math
import os
import shutil
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import List, Optional, Sequence, Tuple, Union

from fault_detector_msgs.msg import TagElement
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image

from .models import InspectionObject, ReferenceView
from .object_repository import ObjectRepository
from .reference_camera_registry import REFERENCE_CAMERA_BY_ID
from .reference_view_depth_projection import rgb_depth_overlap_region
from .reference_view_validation import validate_reference_view_inputs
from .repository_utils import atomic_write_text, validate_storage_name


@dataclass(frozen=True)
class CapturedReferenceView:
    """One camera-specific synchronized reference dataset."""

    slot_index: int
    camera_id: str
    reference_view: ReferenceView
    rgb_image: Image
    depth_image: Image
    rgb_camera_info: CameraInfo
    depth_camera_info: CameraInfo
    reference_tag: TagElement
    fixed_frame: str

    @property
    def camera_info(self) -> CameraInfo:
        """Return registered-depth calibration for projection callers."""
        return self.depth_camera_info


class MultiReferenceViewRepository:
    """Store one authoritative versioned reference-view set per routine."""

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
        return bool(routine.reference_views)

    def save_reference_views(
        self,
        object_id: str,
        routine_id: str,
        captures: Sequence[CapturedReferenceView],
        maximum_timestamp_skew_sec: float = 0.05,
    ) -> InspectionObject:
        """Replace a routine's complete view set as one transaction."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")

        definition = self.object_repository.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")
        normalized = self._validate_captures(
            captures,
            definition.reference_tag.tag_id,
            maximum_timestamp_skew_sec,
        )

        routine_root = (
            self.object_repository.get_object_dir(object_id)
            / self.object_repository.REFERENCE_DATASET_DIRECTORY
            / routine_id
        )
        routine_root.mkdir(parents=True, exist_ok=True)
        capture_set_id = self._capture_set_id(normalized)
        final_set_path = routine_root / capture_set_id
        if final_set_path.exists():
            raise FileExistsError(
                f"Reference-view set already exists: {final_set_path}"
            )
        staging_path = Path(tempfile.mkdtemp(
            prefix=f".{capture_set_id}.",
            suffix=".tmp",
            dir=str(routine_root),
        ))

        published = False
        stored_views = []
        try:
            for capture in normalized:
                view_id = self._view_id(capture)
                view_path = staging_path / view_id
                view_path.mkdir()
                stored_view = replace(
                    capture.reference_view,
                    reference_dataset_path=(
                        Path(
                            self.object_repository
                            .REFERENCE_DATASET_DIRECTORY
                        )
                        / routine_id
                        / capture_set_id
                        / view_id
                    ).as_posix(),
                    view_id=view_id,
                    camera_id=capture.camera_id,
                    slot_index=capture.slot_index,
                )
                stored_view.validate()
                self._write_dataset(
                    view_path,
                    object_id,
                    routine_id,
                    capture_set_id,
                    capture,
                    stored_view,
                    maximum_timestamp_skew_sec,
                )
                stored_views.append(stored_view)

            stored_routine = replace(
                routine,
                reference_views=stored_views,
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

            _fsync_directory(staging_path)
            os.replace(staging_path, final_set_path)
            published = True
            _fsync_directory(routine_root)
            self.object_repository.save(stored_definition)
        except Exception:
            if staging_path.exists():
                shutil.rmtree(staging_path, ignore_errors=True)
            if published:
                shutil.rmtree(final_set_path, ignore_errors=True)
                _fsync_directory(routine_root)
            raise

        self._delete_unreferenced_capture_sets(
            routine_root,
            capture_set_id,
        )
        return stored_definition

    def load_reference_views(
        self,
        object_id: str,
        routine_id: str,
    ) -> List[CapturedReferenceView]:
        """Load all views directly from the authoritative object model."""
        validate_storage_name(object_id, "object ID")
        validate_storage_name(routine_id, "routine ID")
        definition = self.object_repository.load(object_id)
        routine = definition.get_routine(routine_id)
        if routine is None:
            raise KeyError(f"Routine does not exist: {routine_id}")

        result = []
        for reference_view in sorted(
            routine.reference_views,
            key=lambda view: view.slot_index,
        ):
            if reference_view.camera_id not in REFERENCE_CAMERA_BY_ID:
                raise ValueError(
                    "Reference view contains an unknown camera ID: "
                    f"{reference_view.camera_id}"
                )
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
            self._validate_metadata(
                metadata,
                object_id,
                routine_id,
                reference_view,
            )
            rgb_image = deserialize_message(
                (dataset_path / "rgb.cdr").read_bytes(),
                Image,
            )
            depth_image = deserialize_message(
                (dataset_path / "depth.cdr").read_bytes(),
                Image,
            )
            rgb_camera_info = deserialize_message(
                (dataset_path / "rgb_camera_info.cdr").read_bytes(),
                CameraInfo,
            )
            depth_camera_info = deserialize_message(
                (dataset_path / "depth_camera_info.cdr").read_bytes(),
                CameraInfo,
            )
            reference_tag = deserialize_message(
                (dataset_path / "reference_tag.cdr").read_bytes(),
                TagElement,
            )
            self._validate_loaded_dataset(
                metadata,
                reference_view,
                definition.reference_tag.tag_id,
                rgb_image,
                depth_image,
                rgb_camera_info,
                depth_camera_info,
                reference_tag,
            )
            result.append(CapturedReferenceView(
                slot_index=reference_view.slot_index,
                camera_id=reference_view.camera_id,
                reference_view=reference_view,
                rgb_image=rgb_image,
                depth_image=depth_image,
                rgb_camera_info=rgb_camera_info,
                depth_camera_info=depth_camera_info,
                reference_tag=reference_tag,
                fixed_frame=str(metadata["fixed_frame"]),
            ))
        return result

    @staticmethod
    def _validate_captures(
        captures: Sequence[CapturedReferenceView],
        reference_tag_id: int,
        maximum_timestamp_skew_sec: float,
    ) -> Tuple[CapturedReferenceView, ...]:
        if (
            not math.isfinite(maximum_timestamp_skew_sec)
            or maximum_timestamp_skew_sec < 0.0
        ):
            raise ValueError(
                "Maximum timestamp skew must be finite and non-negative"
            )
        normalized = tuple(sorted(captures, key=lambda item: item.slot_index))
        if not 1 <= len(normalized) <= 3:
            raise ValueError(
                "Reference capture must contain one to three views"
            )
        slots = [capture.slot_index for capture in normalized]
        cameras = [capture.camera_id for capture in normalized]
        if any(slot < 0 or slot > 2 for slot in slots):
            raise ValueError("Reference camera slot must be between 0 and 2")
        if len(set(slots)) != len(slots):
            raise ValueError("Reference camera slots must be unique")
        if len(set(cameras)) != len(cameras):
            raise ValueError("Reference camera IDs must be unique")
        fixed_frames = {
            capture.fixed_frame for capture in normalized
        }
        if len(fixed_frames) != 1:
            raise ValueError(
                "Reference captures must use one fixed frame"
            )
        reference_tags = {
            _tag_signature(capture.reference_tag)
            for capture in normalized
        }
        if len(reference_tags) != 1:
            raise ValueError(
                "Reference captures must use one shared tag observation"
            )
        for capture in normalized:
            if not capture.camera_id.strip():
                raise ValueError("Reference camera ID must not be empty")
            if capture.camera_id not in REFERENCE_CAMERA_BY_ID:
                raise ValueError(
                    "Reference capture contains an unknown camera ID: "
                    f"{capture.camera_id}"
                )
            if not capture.fixed_frame.strip():
                raise ValueError("Reference fixed frame must not be empty")
            capture.reference_view.validate()
            if capture.reference_view.reference_dataset_path is not None:
                raise ValueError(
                    "New reference view already has a dataset path"
                )
            image_skew_sec = abs(
                _message_stamp_nanoseconds(capture.rgb_image)
                - _message_stamp_nanoseconds(capture.depth_image)
            ) / 1_000_000_000
            validate_reference_view_inputs(
                capture.rgb_image,
                capture.depth_image,
                capture.depth_camera_info,
                capture.reference_tag,
                reference_tag_id,
                capture.reference_view.controlled_frame_pose_object,
                capture.reference_view.controlled_frame,
                maximum_timestamp_skew_sec=(
                    maximum_timestamp_skew_sec
                ),
                rgb_camera_info=capture.rgb_camera_info,
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
            or len(relative.parts) != 4
            or relative.parts[:2] != (
                self.object_repository.REFERENCE_DATASET_DIRECTORY,
                routine_id,
            )
            or relative.parts[-1] != reference_view.view_id
        ):
            raise ValueError(
                "Reference dataset path is outside the selected routine"
            )
        return self.object_repository.get_object_dir(object_id) / relative

    @staticmethod
    def _capture_set_id(
        captures: Sequence[CapturedReferenceView],
    ) -> str:
        stamp = max(
            (
                capture.rgb_image.header.stamp.sec,
                capture.rgb_image.header.stamp.nanosec,
            )
            for capture in captures
        )
        _validate_stamp(stamp[0], stamp[1], "RGB image")
        return f"set_{stamp[0]}_{stamp[1]:09d}"

    @staticmethod
    def _view_id(capture: CapturedReferenceView) -> str:
        return f"slot{capture.slot_index + 1}_{capture.camera_id}"

    @staticmethod
    def _write_dataset(
        path: Path,
        object_id: str,
        routine_id: str,
        capture_set_id: str,
        capture: CapturedReferenceView,
        stored_view: ReferenceView,
        maximum_timestamp_skew_sec: float,
    ) -> None:
        files = {
            "rgb.cdr": bytes(serialize_message(capture.rgb_image)),
            "depth.cdr": bytes(serialize_message(capture.depth_image)),
            "rgb_camera_info.cdr": bytes(
                serialize_message(capture.rgb_camera_info)
            ),
            "depth_camera_info.cdr": bytes(
                serialize_message(capture.depth_camera_info)
            ),
            "reference_tag.cdr": bytes(
                serialize_message(capture.reference_tag)
            ),
        }
        for name, content in files.items():
            file_path = path / name
            with file_path.open("wb") as output:
                output.write(content)
                output.flush()
                os.fsync(output.fileno())

        rgb_stamp = _stamp_dict(capture.rgb_image.header.stamp)
        depth_stamp = _stamp_dict(capture.depth_image.header.stamp)
        tag_stamp = _stamp_dict(
            capture.reference_tag.pose.header.stamp
        )
        overlap = rgb_depth_overlap_region(
            (
                capture.rgb_image.width,
                capture.rgb_image.height,
            ),
            (
                capture.depth_image.width,
                capture.depth_image.height,
            ),
            capture.rgb_camera_info,
            capture.depth_camera_info,
        )
        metadata = {
            "object_id": object_id,
            "routine_id": routine_id,
            "capture_set_id": capture_set_id,
            "view_id": stored_view.view_id,
            "camera_id": capture.camera_id,
            "slot_index": capture.slot_index,
            "fixed_frame": capture.fixed_frame,
            "controlled_frame": stored_view.controlled_frame,
            "controlled_frame_pose_object": (
                stored_view.controlled_frame_pose_object.to_dict()
            ),
            "rgb": _image_metadata(capture.rgb_image, rgb_stamp),
            "depth": _image_metadata(
                capture.depth_image,
                depth_stamp,
            ),
            "rgb_camera_info": _camera_info_metadata(
                capture.rgb_camera_info
            ),
            "depth_camera_info": _camera_info_metadata(
                capture.depth_camera_info
            ),
            "reference_tag": {
                "id": int(capture.reference_tag.id),
                "frame_id": (
                    capture.reference_tag.pose.header.frame_id
                ),
                "timestamp": tag_stamp,
            },
            "rgb_depth_skew_sec": abs(
                _stamp_nanoseconds(rgb_stamp)
                - _stamp_nanoseconds(depth_stamp)
            ) / 1_000_000_000,
            "maximum_rgb_depth_skew_sec": (
                maximum_timestamp_skew_sec
            ),
            "rgb_tag_skew_sec": abs(
                _stamp_nanoseconds(rgb_stamp)
                - _stamp_nanoseconds(tag_stamp)
            ) / 1_000_000_000,
            "rgb_depth_overlap_region": overlap.to_dict(),
        }
        atomic_write_text(
            path / "metadata.json",
            json.dumps(metadata, indent=2) + "\n",
        )
        _fsync_directory(path)

    @staticmethod
    def _validate_metadata(
        metadata,
        object_id,
        routine_id,
        reference_view,
    ) -> None:
        expected = {
            "object_id": object_id,
            "routine_id": routine_id,
            "view_id": reference_view.view_id,
            "camera_id": reference_view.camera_id,
            "slot_index": reference_view.slot_index,
        }
        for name, value in expected.items():
            if metadata.get(name) != value:
                raise ValueError(
                    f"Reference dataset {name} does not match"
                )
        if (
            metadata.get("controlled_frame")
            != reference_view.controlled_frame
        ):
            raise ValueError(
                "Reference dataset controlled frame does not match"
            )
        if (
            metadata.get("controlled_frame_pose_object")
            != reference_view.controlled_frame_pose_object.to_dict()
        ):
            raise ValueError(
                "Reference dataset camera pose does not match"
            )

    @staticmethod
    def _validate_loaded_dataset(
        metadata,
        reference_view,
        reference_tag_id,
        rgb_image,
        depth_image,
        rgb_camera_info,
        depth_camera_info,
        reference_tag,
    ) -> None:
        rgb_stamp = _stamp_dict(rgb_image.header.stamp)
        depth_stamp = _stamp_dict(depth_image.header.stamp)
        tag_stamp = _stamp_dict(reference_tag.pose.header.stamp)
        expected_messages = {
            "rgb": _image_metadata(rgb_image, rgb_stamp),
            "depth": _image_metadata(depth_image, depth_stamp),
            "rgb_camera_info": _camera_info_metadata(
                rgb_camera_info
            ),
            "depth_camera_info": _camera_info_metadata(
                depth_camera_info
            ),
            "reference_tag": {
                "id": int(reference_tag.id),
                "frame_id": reference_tag.pose.header.frame_id,
                "timestamp": tag_stamp,
            },
        }
        for name, value in expected_messages.items():
            if metadata.get(name) != value:
                raise ValueError(
                    f"Reference dataset {name} metadata does not match"
                )
        image_skew_sec = abs(
            _stamp_nanoseconds(rgb_stamp)
            - _stamp_nanoseconds(depth_stamp)
        ) / 1_000_000_000
        stored_image_skew = float(metadata["rgb_depth_skew_sec"])
        stored_maximum_image_skew = float(
            metadata["maximum_rgb_depth_skew_sec"]
        )
        if (
            not math.isfinite(stored_image_skew)
            or not math.isclose(
                stored_image_skew,
                image_skew_sec,
                abs_tol=1e-12,
            )
        ):
            raise ValueError(
                "Reference dataset RGB-depth skew does not match"
            )
        if (
            not math.isfinite(stored_maximum_image_skew)
            or stored_maximum_image_skew < 0.0
            or image_skew_sec > stored_maximum_image_skew
        ):
            raise ValueError(
                "Reference dataset RGB-depth skew exceeds its limit"
            )
        validate_reference_view_inputs(
            rgb_image,
            depth_image,
            depth_camera_info,
            reference_tag,
            reference_tag_id,
            reference_view.controlled_frame_pose_object,
            reference_view.controlled_frame,
            maximum_timestamp_skew_sec=stored_maximum_image_skew,
            rgb_camera_info=rgb_camera_info,
        )
        overlap = rgb_depth_overlap_region(
            (rgb_image.width, rgb_image.height),
            (depth_image.width, depth_image.height),
            rgb_camera_info,
            depth_camera_info,
        )
        if (
            metadata.get("rgb_depth_overlap_region")
            != overlap.to_dict()
        ):
            raise ValueError(
                "Reference dataset RGB-depth overlap does not match"
            )
        fixed_frame = metadata.get("fixed_frame")
        if not isinstance(fixed_frame, str) or not fixed_frame.strip():
            raise ValueError(
                "Reference dataset fixed frame must not be empty"
            )

    @staticmethod
    def _delete_unreferenced_capture_sets(
        routine_root: Path,
        referenced_set_id: str,
    ) -> None:
        for child in routine_root.iterdir():
            if (
                child.is_dir()
                and not child.name.startswith(".")
                and child.name != referenced_set_id
            ):
                shutil.rmtree(child, ignore_errors=True)
        _fsync_directory(routine_root)


def _validate_stamp(sec: int, nanosec: int, name: str) -> None:
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f"{name} timestamp is invalid")
    if sec == 0 and nanosec == 0:
        raise ValueError(f"{name} timestamp must not be zero")


def _stamp_dict(stamp) -> dict:
    _validate_stamp(stamp.sec, stamp.nanosec, "Sensor input")
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _stamp_nanoseconds(stamp: dict) -> int:
    return stamp["sec"] * 1_000_000_000 + stamp["nanosec"]


def _message_stamp_nanoseconds(message) -> int:
    stamp = message.header.stamp
    _validate_stamp(stamp.sec, stamp.nanosec, "Sensor input")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _tag_signature(tag: TagElement) -> tuple:
    pose = tag.pose.pose
    return (
        int(tag.id),
        tag.pose.header.frame_id,
        _message_stamp_nanoseconds(tag.pose),
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def _image_metadata(image: Image, stamp: dict) -> dict:
    return {
        "frame_id": image.header.frame_id,
        "timestamp": stamp,
        "width": int(image.width),
        "height": int(image.height),
        "encoding": image.encoding,
        "step": int(image.step),
    }


def _camera_info_metadata(camera_info: CameraInfo) -> dict:
    return {
        "frame_id": camera_info.header.frame_id,
        "timestamp": {
            "sec": int(camera_info.header.stamp.sec),
            "nanosec": int(camera_info.header.stamp.nanosec),
        },
        "width": int(camera_info.width),
        "height": int(camera_info.height),
        "distortion_model": camera_info.distortion_model,
        "d": [float(value) for value in camera_info.d],
        "k": [float(value) for value in camera_info.k],
        "r": [float(value) for value in camera_info.r],
        "p": [float(value) for value in camera_info.p],
        "binning_x": int(camera_info.binning_x),
        "binning_y": int(camera_info.binning_y),
        "roi": {
            "x_offset": int(camera_info.roi.x_offset),
            "y_offset": int(camera_info.roi.y_offset),
            "height": int(camera_info.roi.height),
            "width": int(camera_info.roi.width),
            "do_rectify": bool(camera_info.roi.do_rectify),
        },
    }


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
