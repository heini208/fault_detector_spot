"""Synchronized hand-camera and base-tag reference-view inputs."""

import math
from copy import deepcopy
from threading import Lock
from typing import Dict, Optional, Tuple

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
    """Retain the latest camera inputs and base-tag observations."""

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
        self._base_tags: Dict[int, TagElement] = {}

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
    ) -> Optional[ReferenceViewInputs]:
        """Return isolated camera inputs and the selected base tag."""
        if reference_tag_id < 0:
            raise ValueError("Reference tag ID must not be negative")

        with self._lock:
            tag = self._base_tags.get(reference_tag_id)
            if (
                self._latest_images is None
                or self._camera_info is None
                or tag is None
            ):
                return None

            rgb_image, depth_image = self._latest_images
            return deepcopy(
                (rgb_image, depth_image, self._camera_info, tag)
            )

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
            for observation in observations.elements:
                tag_id = int(observation.id)
                existing = self._base_tags.get(tag_id)
                if (
                    existing is not None
                    and self._stamp_key(observation)
                    < self._stamp_key(existing)
                ):
                    continue
                self._base_tags[tag_id] = deepcopy(observation)

    def _synchronized_images_callback(
        self,
        rgb_image: Image,
        depth_image: Image,
    ) -> None:
        with self._lock:
            self._latest_images = (
                deepcopy(rgb_image),
                deepcopy(depth_image),
            )

    @staticmethod
    def _stamp_key(observation: TagElement) -> Tuple[int, int]:
        stamp = observation.pose.header.stamp
        return stamp.sec, stamp.nanosec
