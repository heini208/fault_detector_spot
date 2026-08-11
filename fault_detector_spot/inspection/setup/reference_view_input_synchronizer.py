"""Camera-specific collection and exhaustive RGB-depth matching."""

import math
import time
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from threading import Lock
from typing import Deque, Optional, Tuple

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


ReferenceViewInputs = Tuple[
    Image,
    Image,
    CameraInfo,
    CameraInfo,
]
ImageSample = Tuple[int, Image]


@dataclass
class _CollectionWindow:
    minimum_input_sequence: int
    ends_at_nanoseconds: int
    rgb_images: Deque[ImageSample]
    depth_images: Deque[ImageSample]
    rgb_camera_info: Optional[CameraInfo] = None
    depth_camera_info: Optional[CameraInfo] = None


class ReferenceViewInputSynchronizer:
    """Buffer raw camera inputs and choose the closest valid pair."""

    def __init__(
        self,
        node: Node,
        rgb_topic: str,
        depth_topic: str,
        rgb_camera_info_topic: str,
        depth_camera_info_topic: str,
        queue_size: int = 10,
        maximum_timestamp_skew_sec: float = 0.05,
        collection_duration_sec: float = 1.0,
    ):
        """Subscribe to one selected camera's raw input streams."""
        if (
            isinstance(queue_size, bool)
            or not isinstance(queue_size, int)
            or queue_size < 1
        ):
            raise ValueError("Synchronization queue size must be positive")
        self._validate_duration(
            maximum_timestamp_skew_sec,
            "Maximum timestamp skew",
            allow_zero=True,
        )
        self._validate_duration(
            collection_duration_sec,
            "Collection duration",
            allow_zero=False,
        )

        self._lock = Lock()
        self._rgb_camera_info: Optional[CameraInfo] = None
        self._depth_camera_info: Optional[CameraInfo] = None
        self._input_sequence = 0
        self._idle_history_size = 2
        self._collection_history_size = queue_size
        self._rgb_history: Deque[ImageSample] = deque(
            maxlen=self._idle_history_size
        )
        self._depth_history: Deque[ImageSample] = deque(
            maxlen=self._idle_history_size
        )
        self._maximum_timestamp_skew_nanoseconds = int(
            maximum_timestamp_skew_sec * 1_000_000_000
        )
        self._collection_duration_nanoseconds = int(
            collection_duration_sec * 1_000_000_000
        )
        self._collection: Optional[_CollectionWindow] = None

        self.rgb_subscription = node.create_subscription(
            Image,
            rgb_topic,
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.depth_subscription = node.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.rgb_camera_info_subscription = node.create_subscription(
            CameraInfo,
            rgb_camera_info_topic,
            self._rgb_camera_info_callback,
            qos_profile_sensor_data,
        )
        self.depth_camera_info_subscription = node.create_subscription(
            CameraInfo,
            depth_camera_info_topic,
            self._depth_camera_info_callback,
            qos_profile_sensor_data,
        )

    def ready_for_collection(self) -> bool:
        """Return whether both image streams and calibrations are warm."""
        with self._lock:
            return (
                self._rgb_camera_info is not None
                and self._depth_camera_info is not None
                and bool(self._rgb_history)
                and bool(self._depth_history)
            )

    def begin_collection(self, minimum_input_sequence: int) -> None:
        """Start one bounded camera-specific capture window."""
        self._validate_sequence(minimum_input_sequence)
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            if self._collection is not None:
                raise RuntimeError(
                    "A reference-view collection is already active"
                )
            self._collection = _CollectionWindow(
                minimum_input_sequence=minimum_input_sequence,
                ends_at_nanoseconds=(
                    now_nanoseconds
                    + self._collection_duration_nanoseconds
                ),
                rgb_images=deque(
                    (
                        sample for sample in self._rgb_history
                        if sample[0] >= minimum_input_sequence
                    ),
                    maxlen=self._collection_history_size,
                ),
                depth_images=deque(
                    (
                        sample for sample in self._depth_history
                        if sample[0] >= minimum_input_sequence
                    ),
                    maxlen=self._collection_history_size,
                ),
                rgb_camera_info=deepcopy(self._rgb_camera_info),
                depth_camera_info=deepcopy(self._depth_camera_info),
            )

    def collection_ready(self) -> bool:
        """Return whether the active collection window has completed."""
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            return (
                self._collection is not None
                and now_nanoseconds
                >= self._collection.ends_at_nanoseconds
            )

    def best_snapshot(
        self,
        minimum_input_sequence: int,
    ) -> Optional[ReferenceViewInputs]:
        """Return the globally closest RGB-depth pair in the window."""
        self._validate_sequence(minimum_input_sequence)
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            collection = self._collection
            if collection is None:
                raise RuntimeError(
                    "No reference-view collection is active"
                )
            if (
                collection.minimum_input_sequence
                != minimum_input_sequence
            ):
                raise RuntimeError(
                    "The active collection does not match the request"
                )
            if now_nanoseconds < collection.ends_at_nanoseconds:
                raise RuntimeError(
                    "Reference-view collection is not complete"
                )
            if (
                collection.rgb_camera_info is None
                or collection.depth_camera_info is None
            ):
                return None
            rgb_images = tuple(collection.rgb_images)
            depth_images = tuple(collection.depth_images)
            rgb_camera_info = deepcopy(collection.rgb_camera_info)
            depth_camera_info = deepcopy(collection.depth_camera_info)

        selected = self._select_best_pair(rgb_images, depth_images)
        if selected is None:
            return None
        rgb_image, depth_image = selected
        return (
            deepcopy(rgb_image),
            deepcopy(depth_image),
            rgb_camera_info,
            depth_camera_info,
        )

    def input_diagnostics(self) -> str:
        """Describe the current warmup state for this camera."""
        with self._lock:
            return self._format_diagnostics(
                rgb_camera_info_available=(
                    self._rgb_camera_info is not None
                ),
                depth_camera_info_available=(
                    self._depth_camera_info is not None
                ),
                rgb_images=tuple(self._rgb_history),
                depth_images=tuple(self._depth_history),
            )

    def collection_diagnostics(
        self,
        minimum_input_sequence: int,
    ) -> str:
        """Describe why a completed camera window had no result."""
        with self._lock:
            collection = self._collection
            if collection is None:
                return self._format_diagnostics(
                    rgb_camera_info_available=(
                        self._rgb_camera_info is not None
                    ),
                    depth_camera_info_available=(
                        self._depth_camera_info is not None
                    ),
                    rgb_images=tuple(self._rgb_history),
                    depth_images=tuple(self._depth_history),
                )
            if (
                collection.minimum_input_sequence
                != minimum_input_sequence
            ):
                return "active collection does not match the request"
            return self._format_diagnostics(
                rgb_camera_info_available=(
                    collection.rgb_camera_info is not None
                ),
                depth_camera_info_available=(
                    collection.depth_camera_info is not None
                ),
                rgb_images=tuple(collection.rgb_images),
                depth_images=tuple(collection.depth_images),
            )

    def cancel_collection(self) -> None:
        """Discard the active collection."""
        with self._lock:
            self._collection = None

    @property
    def input_sequence(self) -> int:
        """Return the number of raw image messages received."""
        with self._lock:
            return self._input_sequence

    @property
    def image_sequence(self) -> int:
        """Return the raw input sequence retained for older callers."""
        return self.input_sequence

    @property
    def collection_active(self) -> bool:
        """Return whether a camera collection is active."""
        with self._lock:
            return self._collection is not None

    @property
    def latest_rgb_frame_id(self) -> str:
        """Return the newest RGB frame ID."""
        with self._lock:
            if not self._rgb_history:
                return ""
            return self._rgb_history[-1][1].header.frame_id

    def _rgb_camera_info_callback(
        self,
        camera_info: CameraInfo,
    ) -> None:
        with self._lock:
            self._rgb_camera_info = deepcopy(camera_info)

    def _depth_camera_info_callback(
        self,
        camera_info: CameraInfo,
    ) -> None:
        with self._lock:
            self._depth_camera_info = deepcopy(camera_info)

    def _rgb_callback(self, image: Image) -> None:
        self._append_image(image, is_rgb=True)

    def _depth_callback(self, image: Image) -> None:
        self._append_image(image, is_rgb=False)

    def _append_image(self, image: Image, is_rgb: bool) -> None:
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            self._input_sequence += 1
            sample = (self._input_sequence, image)
            history = (
                self._rgb_history if is_rgb else self._depth_history
            )
            history.append(sample)
            collection = self._collection
            if (
                collection is None
                or sample[0] < collection.minimum_input_sequence
                or now_nanoseconds > collection.ends_at_nanoseconds
            ):
                return
            target = (
                collection.rgb_images
                if is_rgb
                else collection.depth_images
            )
            target.append(sample)

    def _synchronized_images_callback(
        self,
        rgb_image: Image,
        depth_image: Image,
    ) -> None:
        self._rgb_callback(rgb_image)
        self._depth_callback(depth_image)

    def _select_best_pair(
        self,
        rgb_images,
        depth_images,
    ) -> Optional[Tuple[Image, Image]]:
        best_candidate = None
        best_score = None
        for rgb_sequence, rgb_image in rgb_images:
            rgb_stamp = self._image_stamp_nanoseconds(rgb_image)
            if rgb_stamp <= 0:
                continue
            for depth_sequence, depth_image in depth_images:
                depth_stamp = self._image_stamp_nanoseconds(depth_image)
                if depth_stamp <= 0:
                    continue
                skew = abs(rgb_stamp - depth_stamp)
                if skew > self._maximum_timestamp_skew_nanoseconds:
                    continue
                score = (
                    skew,
                    -max(rgb_stamp, depth_stamp),
                    -rgb_sequence,
                    -depth_sequence,
                )
                if best_score is None or score < best_score:
                    best_candidate = (rgb_image, depth_image)
                    best_score = score
        return best_candidate

    def _format_diagnostics(
        self,
        rgb_camera_info_available,
        depth_camera_info_available,
        rgb_images,
        depth_images,
    ) -> str:
        closest_skew = self._closest_image_skew(
            rgb_images,
            depth_images,
        )
        candidate = self._select_best_pair(
            rgb_images,
            depth_images,
        )
        return (
            f"rgb_camera_info="
            f"{'yes' if rgb_camera_info_available else 'no'}, "
            f"depth_camera_info="
            f"{'yes' if depth_camera_info_available else 'no'}, "
            f"rgb_frames={len(rgb_images)}, "
            f"depth_frames={len(depth_images)}, "
            f"closest_rgb_depth_skew="
            f"{self._format_skew(closest_skew)}, "
            f"valid_pair={'yes' if candidate else 'no'}"
        )

    def _closest_image_skew(
        self,
        rgb_images,
        depth_images,
    ) -> Optional[int]:
        skews = []
        for _, rgb_image in rgb_images:
            rgb_stamp = self._image_stamp_nanoseconds(rgb_image)
            if rgb_stamp <= 0:
                continue
            for _, depth_image in depth_images:
                depth_stamp = self._image_stamp_nanoseconds(depth_image)
                if depth_stamp > 0:
                    skews.append(abs(rgb_stamp - depth_stamp))
        return min(skews) if skews else None

    @staticmethod
    def _format_skew(skew_nanoseconds: Optional[int]) -> str:
        if skew_nanoseconds is None:
            return "n/a"
        return f"{skew_nanoseconds / 1_000_000:.1f}ms"

    @staticmethod
    def _validate_duration(
        value: float,
        name: str,
        allow_zero: bool,
    ) -> None:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        if value < 0.0 or (not allow_zero and value == 0.0):
            qualifier = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be {qualifier}")

    @staticmethod
    def _validate_sequence(minimum_input_sequence: int) -> None:
        if minimum_input_sequence < 0:
            raise ValueError(
                "Minimum input sequence must not be negative"
            )

    @staticmethod
    def _image_stamp_nanoseconds(image: Image) -> int:
        stamp = image.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec
