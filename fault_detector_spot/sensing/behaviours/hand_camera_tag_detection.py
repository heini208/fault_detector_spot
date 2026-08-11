"""Resolve hand-camera AprilTag detections into body-frame poses."""

from collections import deque
from copy import deepcopy
from typing import Deque, Dict, Optional

import py_trees
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Time as TimeMessage
from fault_detector_msgs.msg import TagElement
from rclpy.node import Node
from rclpy.time import Time

from fault_detector_spot.sensing.data.tag_observation_cache import (
    TagObservationCache,
)
from fault_detector_spot.sensing.data.tag_observation_time import (
    is_observation_fresh,
)


class HandCameraTagDetection(py_trees.behaviour.Behaviour):
    """Cache valid hand detections and retry delayed TF transforms."""

    def __init__(
        self,
        name: str = "HandCameraTagDetection",
        detection_topic: str = "/detections",
        target_frame: str = "body",
        tag_frame_prefix: str = "tag36h11:",
        max_age_sec: float = 1.0,
        tf_pending_sec: float = 0.5,
        max_hamming: int = 0,
        min_decision_margin: float = 0.0,
        pending_queue_size: int = 10,
    ):
        """Create the hand-camera detection behaviour."""
        super().__init__(name)

        if tf_pending_sec < 0.0:
            raise ValueError("tf_pending_sec must be non-negative")

        if pending_queue_size < 1:
            raise ValueError("pending_queue_size must be positive")

        self.node: Optional[Node] = None
        self.blackboard = self.attach_blackboard_client()
        self.detection_topic = detection_topic
        self.target_frame = target_frame
        self.tag_frame_prefix = tag_frame_prefix
        self.max_age_sec = max_age_sec
        self.tf_pending_sec = tf_pending_sec
        self.max_hamming = max_hamming
        self.min_decision_margin = min_decision_margin
        self.pending_queue_size = pending_queue_size
        self.pending_detections: Dict[
            int,
            Deque[TimeMessage],
        ] = {}
        self.observation_cache = TagObservationCache(
            max_age_sec=max_age_sec,
        )
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[
            tf2_ros.TransformListener
        ] = None
        self.detection_subscription = None

    def setup(self, **kwargs):
        """Create ROS resources and register blackboard outputs."""
        self.node = kwargs.get("node")

        if self.node is None:
            raise RuntimeError(
                f"{self.name}: no ROS node provided"
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
        )

        self.detection_subscription = (
            self.node.create_subscription(
                AprilTagDetectionArray,
                self.detection_topic,
                self._detections_callback,
                10,
            )
        )

        self.blackboard.register_key(
            key="hand_tag_observations",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.hand_tag_observations = {}
        self.pending_detections.clear()
        self.observation_cache.clear()

    def _detections_callback(
        self,
        message: AprilTagDetectionArray,
    ) -> None:
        """Queue valid detections without clearing state on empty arrays."""
        for detection in message.detections:
            if int(detection.hamming) > self.max_hamming:
                continue

            if (
                float(detection.decision_margin)
                < self.min_decision_margin
            ):
                continue

            tag_id = int(detection.id)
            pending = self.pending_detections.setdefault(
                tag_id,
                deque(maxlen=self.pending_queue_size),
            )
            stamp = deepcopy(message.header.stamp)

            if pending and pending[-1] == stamp:
                continue

            pending.append(stamp)

    def update(self) -> py_trees.common.Status:
        """Resolve pending transforms and publish the fresh hand cache."""
        now = self.node.get_clock().now()
        resolved = self._resolve_pending_observations(now)
        self.observation_cache.update(resolved)
        observations = self.observation_cache.snapshot(now)

        self.blackboard.hand_tag_observations = observations
        self.feedback_message = (
            f"Hand tags: {sorted(observations.keys())}; "
            f"pending: {sorted(self.pending_detections.keys())}"
        )
        return py_trees.common.Status.SUCCESS

    def _resolve_pending_observations(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        """Retry exact-time TF lookup for queued detections."""
        if self.tf_buffer is None or self.node is None:
            return {}

        observations = {}

        for tag_id in list(self.pending_detections):
            pending = self.pending_detections[tag_id]
            fresh_stamps = [
                stamp
                for stamp in pending
                if is_observation_fresh(
                    current_time,
                    stamp,
                    self.tf_pending_sec,
                )
            ]

            if not fresh_stamps:
                del self.pending_detections[tag_id]
                continue

            resolved_index = None

            for index in range(
                len(fresh_stamps) - 1,
                -1,
                -1,
            ):
                stamp = fresh_stamps[index]
                observation_time = Time.from_msg(
                    stamp,
                    clock_type=(
                        self.node.get_clock().clock_type
                    ),
                )
                tag_frame = (
                    f"{self.tag_frame_prefix}{tag_id}"
                )

                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        tag_frame,
                        observation_time,
                    )
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as exception:
                    self.logger.debug(
                        f"Could not resolve {tag_frame} at "
                        f"{stamp.sec}.{stamp.nanosec:09d}: "
                        f"{exception}"
                    )
                    continue

                observations[tag_id] = self._create_tag_element(
                    tag_id,
                    transform,
                    stamp,
                )
                resolved_index = index
                break

            if resolved_index is None:
                self.pending_detections[tag_id] = deque(
                    fresh_stamps,
                    maxlen=self.pending_queue_size,
                )
                continue

            unresolved_newer_stamps = fresh_stamps[
                resolved_index + 1:
            ]

            if unresolved_newer_stamps:
                self.pending_detections[tag_id] = deque(
                    unresolved_newer_stamps,
                    maxlen=self.pending_queue_size,
                )
            else:
                del self.pending_detections[tag_id]

        return observations

    @staticmethod
    def _create_tag_element(
        tag_id: int,
        transform,
        observation_stamp: TimeMessage,
    ) -> TagElement:
        """Convert a resolved transform into a tag observation."""
        tag = TagElement()
        tag.id = tag_id
        tag.pose.header.frame_id = (
            transform.header.frame_id
        )
        tag.pose.header.stamp = observation_stamp
        tag.pose.pose.position.x = (
            transform.transform.translation.x
        )
        tag.pose.pose.position.y = (
            transform.transform.translation.y
        )
        tag.pose.pose.position.z = (
            transform.transform.translation.z
        )
        tag.pose.pose.orientation = (
            transform.transform.rotation
        )
        return tag
