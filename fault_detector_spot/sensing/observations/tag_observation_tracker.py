"""Track fresh base-camera and hand-camera AprilTag observations."""

from collections import deque
from copy import deepcopy
import re
from threading import RLock
from typing import Deque, Dict

import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Time as TimeMessage
from fault_detector_msgs.msg import TagElement
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.sensing.observations.tag_observation_cache import (
    TagObservationCache,
)
from fault_detector_spot.sensing.observations.tag_observation_time import (
    is_observation_fresh,
)


class BaseTagObservationTracker:
    """Resolve raw base-camera visibility and filtered tag geometry."""

    def __init__(
        self,
        tf_buffer,
        frame_pattern: str = r"(?<!filtered_)fiducial_(\d+)",
        filtered_frame_prefix: str = "filtered_fiducial_",
        target_frame: str = "body",
        max_age_sec: float = 1.5,
        logger=None,
    ):
        self.tf_buffer = tf_buffer
        self.frame_pattern = re.compile(frame_pattern)
        self.filtered_frame_prefix = filtered_frame_prefix
        self.target_frame = target_frame
        self.max_age_sec = float(max_age_sec)
        self.logger = logger
        self.observation_cache = TagObservationCache(
            max_age_sec=self.max_age_sec,
        )
        self._frame_lock = RLock()
        self._raw_frames_by_tag = {}

    def receive_tf_frames(self, message: TFMessage) -> None:
        discovered = {}
        for transform in message.transforms:
            match = self.frame_pattern.search(transform.child_frame_id)
            if match is None:
                continue
            discovered[int(match.group(1))] = match.group(0)
        if not discovered:
            return
        with self._frame_lock:
            self._raw_frames_by_tag.update(discovered)

    def snapshot(self, current_time: Time) -> Dict[int, TagElement]:
        self.observation_cache.update(
            self._resolve_observations(current_time)
        )
        return self.observation_cache.snapshot(current_time)

    def _resolve_observations(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        observations = {}
        with self._frame_lock:
            frames = tuple(sorted(self._raw_frames_by_tag.items()))

        for tag_id, raw_frame_name in frames:
            filtered_frame_name = (
                f"{self.filtered_frame_prefix}{tag_id}"
            )
            try:
                raw_transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    raw_frame_name,
                    Time(),
                )
                filtered_transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    filtered_frame_name,
                    Time(),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exception:
                if self.logger is not None:
                    self.logger.debug(
                        f"Could not resolve {raw_frame_name} and "
                        f"{filtered_frame_name}: {exception}"
                    )
                continue

            if not is_observation_fresh(
                current_time,
                raw_transform.header.stamp,
                self.max_age_sec,
            ):
                continue

            observations[tag_id] = self._create_tag_element(
                tag_id,
                filtered_transform,
                raw_transform.header.stamp,
            )
        return observations

    @staticmethod
    def _create_tag_element(
        tag_id: int,
        pose_transform,
        observation_stamp,
    ) -> TagElement:
        tag = TagElement()
        tag.id = tag_id
        tag.pose.header = deepcopy(pose_transform.header)
        tag.pose.header.stamp = observation_stamp
        tag.pose.pose.position.x = (
            pose_transform.transform.translation.x
        )
        tag.pose.pose.position.y = (
            pose_transform.transform.translation.y
        )
        tag.pose.pose.position.z = (
            pose_transform.transform.translation.z
        )
        tag.pose.pose.orientation = (
            pose_transform.transform.rotation
        )
        return tag


class HandTagObservationTracker:
    """Resolve timestamped hand-camera detections through shared TF."""

    def __init__(
        self,
        tf_buffer,
        target_frame: str = "body",
        tag_frame_prefix: str = "tag36h11:",
        max_age_sec: float = 1.0,
        tf_pending_sec: float = 0.5,
        max_hamming: int = 0,
        min_decision_margin: float = 0.0,
        pending_queue_size: int = 10,
        logger=None,
    ):
        if tf_pending_sec < 0.0:
            raise ValueError("tf_pending_sec must be non-negative")
        if pending_queue_size < 1:
            raise ValueError("pending_queue_size must be positive")
        self.tf_buffer = tf_buffer
        self.target_frame = target_frame
        self.tag_frame_prefix = tag_frame_prefix
        self.max_age_sec = float(max_age_sec)
        self.tf_pending_sec = float(tf_pending_sec)
        self.max_hamming = int(max_hamming)
        self.min_decision_margin = float(min_decision_margin)
        self.pending_queue_size = int(pending_queue_size)
        self.logger = logger
        self.pending_detections: Dict[
            int,
            Deque[TimeMessage],
        ] = {}
        self.observation_cache = TagObservationCache(
            max_age_sec=self.max_age_sec,
        )
        self._lock = RLock()

    def receive_detections(
        self,
        message: AprilTagDetectionArray,
    ) -> None:
        with self._lock:
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

    def snapshot(self, current_time: Time) -> Dict[int, TagElement]:
        self.observation_cache.update(
            self._resolve_pending_observations(current_time)
        )
        return self.observation_cache.snapshot(current_time)

    def pending_tag_ids(self):
        with self._lock:
            return tuple(sorted(self.pending_detections))

    def _resolve_pending_observations(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        observations = {}
        with self._lock:
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
                for index in range(len(fresh_stamps) - 1, -1, -1):
                    stamp = fresh_stamps[index]
                    observation_time = Time.from_msg(
                        stamp,
                        clock_type=current_time.clock_type,
                    )
                    tag_frame = f"{self.tag_frame_prefix}{tag_id}"
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
                        if self.logger is not None:
                            self.logger.debug(
                                f"Could not resolve {tag_frame} at "
                                f"{stamp.sec}.{stamp.nanosec:09d}: "
                                f"{exception}"
                            )
                        continue

                    observations[tag_id] = (
                        self._create_tag_element(
                            tag_id,
                            transform,
                            stamp,
                        )
                    )
                    resolved_index = index
                    break

                if resolved_index is None:
                    self.pending_detections[tag_id] = deque(
                        fresh_stamps,
                        maxlen=self.pending_queue_size,
                    )
                    continue

                unresolved = fresh_stamps[resolved_index + 1:]
                if unresolved:
                    self.pending_detections[tag_id] = deque(
                        unresolved,
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
        tag = TagElement()
        tag.id = tag_id
        tag.pose.header.frame_id = transform.header.frame_id
        tag.pose.header.stamp = observation_stamp
        tag.pose.pose.position.x = transform.transform.translation.x
        tag.pose.pose.position.y = transform.transform.translation.y
        tag.pose.pose.position.z = transform.transform.translation.z
        tag.pose.pose.orientation = transform.transform.rotation
        return tag


def merge_tag_observations(base_observations, hand_observations):
    """Merge hand observations without replacing authoritative base tags."""
    merged = deepcopy(base_observations)
    for tag_id, observation in hand_observations.items():
        merged.setdefault(tag_id, deepcopy(observation))
    return merged


__all__ = [
    "BaseTagObservationTracker",
    "HandTagObservationTracker",
    "merge_tag_observations",
]
