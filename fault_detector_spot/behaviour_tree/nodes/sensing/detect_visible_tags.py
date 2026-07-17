import re
from typing import Dict, Optional

import py_trees
import rclpy
import tf2_ros
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.behaviour_tree.nodes.sensing.tag_observation_time import (
    is_observation_fresh,
)


class DetectVisibleTags(py_trees.behaviour.Behaviour):

    def __init__(
        self,
        name: str = "DetectVisibleTags",
        frame_pattern: str = r"filtered_fiducial_(\d+)",
        target_frame: str = "body",
        max_age_sec: float = 0.5,
    ):
        super().__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[
            tf2_ros.TransformListener
        ] = None
        self.blackboard = self.attach_blackboard_client()
        self.frame_pattern = re.compile(frame_pattern)
        self.target_frame = target_frame
        self.max_age_sec = max_age_sec

    def setup(self, **kwargs):
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
            key="base_tag_observations",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.base_tag_observations = {}

    def update(self) -> py_trees.common.Status:
        observations = self._get_visible_tags_from_tf()
        self.blackboard.base_tag_observations = observations
        self.feedback_message = (
            f"Base tags: {sorted(observations.keys())}"
        )
        return py_trees.common.Status.SUCCESS

    def _get_visible_tags_from_tf(
        self,
    ) -> Dict[int, TagElement]:
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

        for line in frame_yaml.splitlines():
            for match in self.frame_pattern.finditer(line):
                frame_name = match.group(0)
                tag_id = int(match.group(1))

                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        frame_name,
                        rclpy.time.Time(),
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
                    self.node.get_clock().now(),
                    transform.header.stamp,
                    self.max_age_sec,
                ):
                    continue

                observations[tag_id] = (
                    self._create_tag_element(
                        tag_id,
                        transform,
                    )
                )

        return observations

    @staticmethod
    def _create_tag_element(
        tag_id: int,
        transform,
    ) -> TagElement:
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