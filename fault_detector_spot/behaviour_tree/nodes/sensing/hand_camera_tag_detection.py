from typing import Dict, Optional
from apriltag_msgs.msg import AprilTagDetectionArray
from fault_detector_msgs.msg import TagElement
import py_trees
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from fault_detector_spot.behaviour_tree.nodes.sensing.tag_observation_time import (
    is_observation_fresh,
)


class HandCameraTagDetection(
    py_trees.behaviour.Behaviour
):

    def __init__(
            self,
            name: str = 'HandCameraTagDetection',
            detection_topic: str = '/detections',
            target_frame: str = 'body',
            tag_frame_prefix: str = 'tag36h11:',
            max_age_sec: float = 0.5,
    ):
        super().__init__(name)
        self.node: Optional[Node] = None
        self.blackboard = self.attach_blackboard_client()
        self.detection_topic = detection_topic
        self.target_frame = target_frame
        self.tag_frame_prefix = tag_frame_prefix
        self.max_age_sec = max_age_sec
        self.latest_detections: Optional[
            AprilTagDetectionArray
        ] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[
            tf2_ros.TransformListener
        ] = None

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

        self.node.create_subscription(
            AprilTagDetectionArray,
            self.detection_topic,
            self._detections_callback,
            10,
        )

        self.blackboard.register_key(
            key="hand_tag_observations",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.hand_tag_observations = {}

    def _detections_callback(
            self,
            message: AprilTagDetectionArray,
    ):
        self.latest_detections = message

    def update(self) -> py_trees.common.Status:
        observations = self._resolve_observations()
        self.blackboard.hand_tag_observations = observations
        self.feedback_message = (
            f"Hand tags: {sorted(observations.keys())}"
        )
        return py_trees.common.Status.SUCCESS

    def _resolve_observations(
            self,
    ) -> Dict[int, TagElement]:
        if (
                self.latest_detections is None
                or self.tf_buffer is None
        ):
            return {}

        message = self.latest_detections

        if not is_observation_fresh(
                self.node.get_clock().now(),
                message.header.stamp,
                self.max_age_sec,
        ):
            return {}

        observation_time = Time.from_msg(
            message.header.stamp
        )
        observations = {}

        for detection in message.detections:
            tag_id = int(detection.id)
            tag_frame = (
                f"{self.tag_frame_prefix}{tag_id}"
            )

            if not self.tf_buffer.can_transform(
                    self.target_frame,
                    tag_frame,
                    observation_time,
                    timeout=Duration(seconds=0.0),
            ):
                continue

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
                    f"Could not resolve {tag_frame}: "
                    f"{exception}"
                )
                continue

            observations[tag_id] = (
                self._create_tag_element(
                    tag_id,
                    transform,
                    message.header.stamp,
                )
            )

        return observations

    @staticmethod
    def _create_tag_element(
            tag_id,
            transform,
            observation_stamp,
    ) -> TagElement:
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