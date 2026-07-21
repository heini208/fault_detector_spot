"""Synchronized hand-camera and base-tag reference-view inputs."""

import math
from collections import deque
from copy import deepcopy
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


class ReferenceViewInputSynchronizer:
    """Retain synchronized camera inputs and timestamped base tags."""

    def __init__(
        self,
        node: Node,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        base_tag_topic: str,
        queue_size: int = 10,
        maximum_timestamp_skew_sec: float = 0.05,
    ):
        """Subscribe to the inputs required to teach a reference view."""
        if queue_size < 1:
            raise ValueError("Synchronization queue size must be positive")
        if (
            not math.isfinite(maximum_timestamp_skew_sec)
            or maximum_timestamp_skew_sec < 0.0
        ):
            raise ValueError(
                "Maximum timestamp skew must be finite and non-negative"
            )

        self._lock = Lock()
        self._camera_info: Optional[CameraInfo] = None
        self._latest_images: Optional[Tuple[Image, Image]] = None
        self._image_sequence = 0
        self._tag_history_size = queue_size
        self._base_tags: Dict[int, Deque[TagElement]] = {}
        self._visible_tag_ids: Set[int] = set()

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

    def snapshot(
        self,
        reference_tag_id: int,
        minimum_image_sequence: int = 0,
    ) -> Optional[ReferenceViewInputs]:
        """Return camera inputs and the closest visible base tag."""
        if reference_tag_id < 0:
            raise ValueError("Reference tag ID must not be negative")
        if minimum_image_sequence < 0:
            raise ValueError(
                "Minimum image sequence must not be negative"
            )

        with self._lock:
            if (
                self._image_sequence < minimum_image_sequence
                or self._latest_images is None
                or self._camera_info is None
                or reference_tag_id not in self._visible_tag_ids
            ):
                return None

            rgb_image, depth_image = self._latest_images
            tag = self._closest_tag(
                reference_tag_id,
                rgb_image,
            )
            if tag is None:
                return None

            return deepcopy(
                (rgb_image, depth_image, self._camera_info, tag)
            )

    @property
    def image_sequence(self) -> int:
        """Return the number of synchronized image pairs received."""
        with self._lock:
            return self._image_sequence

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
        with self._lock:
            visible_tag_ids = set()
            for observation in observations.elements:
                tag_id = int(observation.id)
                visible_tag_ids.add(tag_id)
                history = self._base_tags.setdefault(
                    tag_id,
                    deque(maxlen=self._tag_history_size),
                )
                if not history:
                    history.append(deepcopy(observation))
                    continue

                current_key = self._stamp_key(history[-1])
                observation_key = self._stamp_key(observation)
                if observation_key > current_key:
                    history.append(deepcopy(observation))
                elif observation_key == current_key:
                    history[-1] = deepcopy(observation)

            self._visible_tag_ids = visible_tag_ids

    def _synchronized_images_callback(
        self,
        rgb_image: Image,
        depth_image: Image,
    ) -> None:
        with self._lock:
            self._latest_images = (rgb_image, depth_image)
            self._image_sequence += 1

    def _closest_tag(
        self,
        reference_tag_id: int,
        rgb_image: Image,
    ) -> Optional[TagElement]:
        history = self._base_tags.get(reference_tag_id)
        if not history:
            return None

        rgb_stamp = self._image_stamp_nanoseconds(rgb_image)
        if rgb_stamp == 0:
            return history[-1]

        return min(
            history,
            key=lambda observation: abs(
                self._tag_stamp_nanoseconds(observation) - rgb_stamp
            ),
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
