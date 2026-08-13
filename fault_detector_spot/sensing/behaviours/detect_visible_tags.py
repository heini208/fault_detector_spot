"""Detect fresh Spot base-camera fiducials with stable TF geometry."""

from copy import deepcopy
import re
from threading import RLock
from typing import Dict, Optional

import py_trees
import tf2_ros
from fault_detector_msgs.msg import TagElement
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.sensing.observations.tag_observation_cache import (
    TagObservationCache,
)
from fault_detector_spot.sensing.observations.tag_observation_time import (
    is_observation_fresh,
)


class DetectVisibleTags(py_trees.behaviour.Behaviour):
    """Use raw fiducials for visibility and filtered fiducials for pose."""

    def __init__(
        self,
        name: str = "DetectVisibleTags",
        frame_pattern: str = r"(?<!filtered_)fiducial_(\d+)",
        filtered_frame_prefix: str = "filtered_fiducial_",
        target_frame: str = "body",
        max_age_sec: float = 1.5,
    ):
        """Create the base-camera fiducial behaviour."""
        super().__init__(name)
        self.node: Optional[Node] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[
            tf2_ros.TransformListener
        ] = None
        self.blackboard = self.attach_blackboard_client()
        self.frame_pattern = re.compile(frame_pattern)
        self.filtered_frame_prefix = filtered_frame_prefix
        self.target_frame = target_frame
        self.max_age_sec = max_age_sec
        self.observation_cache = TagObservationCache(
            max_age_sec=max_age_sec,
        )
        self._frame_lock = RLock()
        self._raw_frames_by_tag = {}
        self._tf_frame_subscription = None

    def setup(self, **kwargs):
        """Create TF resources and register blackboard outputs."""
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
        self._tf_frame_subscription = self.node.create_subscription(
            TFMessage,
            "/tf",
            self._receive_tf_frames,
            10,
        )

        self.blackboard.register_key(
            "base_tag_observations",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            "visible_tags",
            access=py_trees.common.Access.WRITE,
        )

        self.blackboard.base_tag_observations = {}
        self.blackboard.visible_tags = {}
        self.observation_cache.clear()
        with self._frame_lock:
            self._raw_frames_by_tag.clear()

    def update(self) -> py_trees.common.Status:
        """Publish the fresh per-tag base-camera cache."""
        now = self.node.get_clock().now()
        new_observations = self._get_visible_tags_from_tf(now)
        self.observation_cache.update(new_observations)
        observations = self.observation_cache.snapshot(now)

        self.blackboard.base_tag_observations = deepcopy(observations)
        self.blackboard.visible_tags = deepcopy(observations)
        self.feedback_message = (
            f"Base tags: {sorted(observations.keys())}"
        )
        return py_trees.common.Status.SUCCESS

    def _receive_tf_frames(self, message: TFMessage) -> None:
        """Remember raw fiducial frame names as TF updates arrive."""
        discovered = {}
        for transform in message.transforms:
            match = self.frame_pattern.search(
                transform.child_frame_id
            )
            if match is None:
                continue
            discovered[int(match.group(1))] = match.group(0)

        if not discovered:
            return

        with self._frame_lock:
            self._raw_frames_by_tag.update(discovered)

    def _raw_fiducial_frames(self):
        with self._frame_lock:
            discovered = tuple(
                sorted(self._raw_frames_by_tag.items())
            )

        if discovered or self._tf_frame_subscription is not None:
            return discovered

        return self._discover_frames_from_buffer()

    def _discover_frames_from_buffer(self):
        if self.tf_buffer is None:
            return ()

        try:
            frame_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception as exception:
            self.logger.warning(
                f"Could not list TF frames: {exception}"
            )
            return ()

        discovered = {}
        for match in self.frame_pattern.finditer(frame_yaml):
            discovered[int(match.group(1))] = match.group(0)

        with self._frame_lock:
            self._raw_frames_by_tag.update(discovered)

        return tuple(sorted(discovered.items()))

    def _get_visible_tags_from_tf(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        """Resolve filtered poses for currently fresh raw fiducials."""
        if self.tf_buffer is None:
            return {}

        observations = {}

        for tag_id, raw_frame_name in self._raw_fiducial_frames():
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
                self.logger.debug(
                    f"Could not resolve {raw_frame_name} and "
                    f"{filtered_frame_name}: "
                    f"{exception}"
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
        """Combine filtered geometry with the raw observation timestamp."""
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
