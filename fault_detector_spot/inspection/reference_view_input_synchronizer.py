"""Synchronized hand-camera and base-tag reference-view inputs."""

import math
import time
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from threading import Lock
from typing import Deque, Dict, Optional, Set, Tuple

from fault_detector_msgs.msg import TagElement, TagElementArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


ReferenceViewInputs = Tuple[
    Image,
    Image,
    CameraInfo,
    TagElement,
]
ImagePair = Tuple[int, Image, Image]


@dataclass
class _CollectionWindow:
    reference_tag_id: int
    minimum_image_sequence: int
    ends_at_nanoseconds: int
    image_pairs: Deque[ImagePair]
    tag_observations: Deque[TagElement]


class ReferenceViewInputSynchronizer:
    """Buffer sensor inputs and select one synchronized capture set."""

    def __init__(
        self,
        node: Node,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        base_tag_topic: str,
        queue_size: int = 10,
        maximum_timestamp_skew_sec: float = 0.05,
        maximum_tag_timestamp_skew_sec: float = 0.25,
        collection_duration_sec: float = 1.0,
    ):
        """Subscribe to all inputs required for reference-view capture."""
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
            maximum_tag_timestamp_skew_sec,
            "Maximum tag timestamp skew",
            allow_zero=True,
        )
        self._validate_duration(
            collection_duration_sec,
            "Collection duration",
            allow_zero=False,
        )

        history_size = max(queue_size, 60)
        self._lock = Lock()
        self._camera_info: Optional[CameraInfo] = None
        self._image_sequence = 0
        self._image_history: Deque[ImagePair] = deque(
            maxlen=history_size
        )
        self._tag_history_size = history_size
        self._base_tags: Dict[int, Deque[TagElement]] = {}
        self._visible_tag_ids: Set[int] = set()
        self._maximum_timestamp_skew_nanoseconds = int(
            maximum_timestamp_skew_sec * 1_000_000_000
        )
        self._maximum_tag_timestamp_skew_nanoseconds = int(
            maximum_tag_timestamp_skew_sec * 1_000_000_000
        )
        self._collection_duration_nanoseconds = int(
            collection_duration_sec * 1_000_000_000
        )
        self._collection: Optional[_CollectionWindow] = None

        self.rgb_subscription = Subscriber(
            node,
            Image,
            rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscription = Subscriber(
            node,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.camera_info_subscription = node.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.base_tag_subscription = node.create_subscription(
            TagElementArray,
            base_tag_topic,
            self._base_tags_callback,
            qos_profile_sensor_data,
        )
        self.image_synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_subscription, self.depth_subscription],
            queue_size,
            maximum_timestamp_skew_sec,
        )
        self.image_synchronizer.registerCallback(
            self._synchronized_images_callback
        )

    def begin_collection(
        self,
        reference_tag_id: int,
        minimum_image_sequence: int,
    ) -> None:
        """Start one bounded reference-view collection transaction."""
        self._validate_request(
            reference_tag_id,
            minimum_image_sequence,
        )
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            if self._collection is not None:
                raise RuntimeError(
                    "A reference-view collection is already active"
                )

            image_pairs = deque(
                (
                    pair
                    for pair in self._image_history
                    if pair[0] >= minimum_image_sequence
                ),
                maxlen=self._image_history.maxlen,
            )
            tag_observations = deque(
                (
                    deepcopy(observation)
                    for observation in self._base_tags.get(
                        reference_tag_id,
                        (),
                    )
                ),
                maxlen=self._tag_history_size,
            )
            self._collection = _CollectionWindow(
                reference_tag_id=reference_tag_id,
                minimum_image_sequence=minimum_image_sequence,
                ends_at_nanoseconds=(
                    now_nanoseconds
                    + self._collection_duration_nanoseconds
                ),
                image_pairs=image_pairs,
                tag_observations=tag_observations,
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
        reference_tag_id: int,
        minimum_image_sequence: int,
    ) -> Optional[ReferenceViewInputs]:
        """Return the best valid set from the completed collection."""
        self._validate_request(
            reference_tag_id,
            minimum_image_sequence,
        )
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            collection = self._collection
            if collection is None:
                raise RuntimeError(
                    "No reference-view collection is active"
                )
            if (
                collection.reference_tag_id != reference_tag_id
                or collection.minimum_image_sequence
                != minimum_image_sequence
            ):
                raise RuntimeError(
                    "The active collection does not match the request"
                )
            if now_nanoseconds < collection.ends_at_nanoseconds:
                raise RuntimeError(
                    "Reference-view collection is not complete"
                )
            if (
                self._camera_info is None
                or reference_tag_id not in self._visible_tag_ids
            ):
                return None

            image_pairs = tuple(collection.image_pairs)
            tag_observations = tuple(
                collection.tag_observations
            )
            camera_info = deepcopy(self._camera_info)

        selected = self._select_best_candidate(
            image_pairs,
            tag_observations,
        )
        if selected is None:
            return None

        rgb_image, depth_image, reference_tag = selected
        return (
            deepcopy(rgb_image),
            deepcopy(depth_image),
            camera_info,
            deepcopy(reference_tag),
        )

    def cancel_collection(self) -> None:
        """Discard the active collection transaction."""
        with self._lock:
            self._collection = None

    @property
    def image_sequence(self) -> int:
        """Return the number of synchronized image pairs received."""
        with self._lock:
            return self._image_sequence

    @property
    def collection_active(self) -> bool:
        """Return whether a collection transaction is active."""
        with self._lock:
            return self._collection is not None

    def _camera_info_callback(
        self,
        camera_info: CameraInfo,
    ) -> None:
        with self._lock:
            self._camera_info = deepcopy(camera_info)

    def _base_tags_callback(
        self,
        observations: TagElementArray,
    ) -> None:
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            visible_tag_ids = set()
            for observation in observations.elements:
                tag_id = int(observation.id)
                visible_tag_ids.add(tag_id)
                history = self._base_tags.setdefault(
                    tag_id,
                    deque(maxlen=self._tag_history_size),
                )
                self._append_tag_observation(
                    history,
                    observation,
                )

                collection = self._collection
                if (
                    collection is not None
                    and tag_id == collection.reference_tag_id
                    and now_nanoseconds
                    <= collection.ends_at_nanoseconds
                ):
                    self._append_tag_observation(
                        collection.tag_observations,
                        observation,
                    )

            self._visible_tag_ids = visible_tag_ids

    def _synchronized_images_callback(
        self,
        rgb_image: Image,
        depth_image: Image,
    ) -> None:
        now_nanoseconds = time.monotonic_ns()
        with self._lock:
            self._image_sequence += 1
            image_pair = (
                self._image_sequence,
                rgb_image,
                depth_image,
            )
            self._image_history.append(image_pair)

            collection = self._collection
            if (
                collection is not None
                and self._image_sequence
                >= collection.minimum_image_sequence
                and now_nanoseconds
                <= collection.ends_at_nanoseconds
            ):
                collection.image_pairs.append(image_pair)

    def _select_best_candidate(
        self,
        image_pairs,
        tag_observations,
    ) -> Optional[Tuple[Image, Image, TagElement]]:
        best_candidate = None
        best_score = None

        for sequence, rgb_image, depth_image in image_pairs:
            rgb_stamp = self._image_stamp_nanoseconds(rgb_image)
            depth_stamp = self._image_stamp_nanoseconds(depth_image)
            if rgb_stamp <= 0 or depth_stamp <= 0:
                continue

            image_skew = abs(rgb_stamp - depth_stamp)
            if (
                image_skew
                > self._maximum_timestamp_skew_nanoseconds
            ):
                continue

            for reference_tag in tag_observations:
                tag_stamp = self._tag_stamp_nanoseconds(
                    reference_tag
                )
                if tag_stamp <= 0:
                    continue

                tag_skew = abs(rgb_stamp - tag_stamp)
                if (
                    tag_skew
                    > self._maximum_tag_timestamp_skew_nanoseconds
                ):
                    continue

                score = (
                    image_skew + tag_skew,
                    max(image_skew, tag_skew),
                    -sequence,
                    -tag_stamp,
                )
                if best_score is None or score < best_score:
                    best_candidate = (
                        rgb_image,
                        depth_image,
                        reference_tag,
                    )
                    best_score = score

        return best_candidate

    @staticmethod
    def _append_tag_observation(
        history: Deque[TagElement],
        observation: TagElement,
    ) -> None:
        if not history:
            history.append(deepcopy(observation))
            return

        current_stamp = ReferenceViewInputSynchronizer._stamp_key(
            history[-1]
        )
        observation_stamp = (
            ReferenceViewInputSynchronizer._stamp_key(
                observation
            )
        )
        if observation_stamp > current_stamp:
            history.append(deepcopy(observation))
        elif observation_stamp == current_stamp:
            history[-1] = deepcopy(observation)

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
    def _validate_request(
        reference_tag_id: int,
        minimum_image_sequence: int,
    ) -> None:
        if reference_tag_id < 0:
            raise ValueError("Reference tag ID must not be negative")
        if minimum_image_sequence < 0:
            raise ValueError(
                "Minimum image sequence must not be negative"
            )

    @staticmethod
    def _image_stamp_nanoseconds(image: Image) -> int:
        stamp = image.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _tag_stamp_nanoseconds(observation: TagElement) -> int:
        stamp = observation.pose.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _stamp_key(observation: TagElement) -> Tuple[int, int]:
        stamp = observation.pose.header.stamp
        return stamp.sec, stamp.nanosec
