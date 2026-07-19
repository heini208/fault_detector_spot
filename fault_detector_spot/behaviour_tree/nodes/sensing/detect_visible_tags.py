"""Detect Spot base-camera fiducials from filtered TF frames."""

import re
from typing import Dict, Optional

import py_trees
import tf2_ros
from fault_detector_msgs.msg import TagElement
from rclpy.node import Node
from rclpy.time import Time
from copy import deepcopy

from .tag_observation_cache import (
    TagObservationCache,
)
from .tag_observation_time import (
    is_observation_fresh,
)


class DetectVisibleTags(py_trees.behaviour.Behaviour):
    """Read filtered base-camera fiducials and retain brief dropouts."""

    def __init__(
        self,
        name: str = "DetectVisibleTags",
        frame_pattern: str = r"filtered_fiducial_(\d+)",
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
        self.target_frame = target_frame
        self.max_age_sec = max_age_sec
        self.observation_cache = TagObservationCache(
            max_age_sec=max_age_sec,
        )

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

    def _get_visible_tags_from_tf(
        self,
        current_time: Time,
    ) -> Dict[int, TagElement]:
        """Resolve currently fresh filtered fiducial TF frames."""
        if self.tf_buffer is None:
            return {}

        try:
            frame_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception as exception:
            self.logger.warning(
                f"Could not list TF frames: {exception}"
            )
            return {}

        observations = {}
        seen_frames = set()

        for match in self.frame_pattern.finditer(frame_yaml):
            frame_name = match.group(0)

            if frame_name in seen_frames:
                continue

            seen_frames.add(frame_name)
            tag_id = int(match.group(1))

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    frame_name,
                    Time(),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exception:
                self.logger.debug(
                    f"Could not resolve {frame_name}: "
                    f"{exception}"
                )
                continue

            if not is_observation_fresh(
                current_time,
                transform.header.stamp,
                self.max_age_sec,
            ):
                continue

            observations[tag_id] = self._create_tag_element(
                tag_id,
                transform,
            )

        return observations

    @staticmethod
    def _create_tag_element(
        tag_id: int,
        transform,
    ) -> TagElement:
        """Convert a TF transform into a tag observation."""
        tag = TagElement()
        tag.id = tag_id
        tag.pose.header = transform.header
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