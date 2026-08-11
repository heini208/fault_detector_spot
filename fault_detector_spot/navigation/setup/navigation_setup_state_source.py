"""Collect runtime pose and tag inputs for navigation authoring."""

from copy import deepcopy
from threading import RLock

import tf2_ros
from fault_detector_msgs.msg import TagElementArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from std_msgs.msg import String

from fault_detector_spot.shared.ros.qos_profiles import (
    LATCHED_QOS,
    LOCALIZATION_POSE_QOS,
)


class NavigationSetupStateSource:
    """Cache localization input and resolve visible tags into map."""

    def __init__(
        self,
        node,
        localization_topic="/localization_pose",
        visible_tag_topic="fault_detector/state/visible_tags",
        active_map_changed=None,
        maximum_pose_age_sec=1.5,
    ):
        self.node = node
        self._lock = RLock()
        self._localization_pose = None
        self._visible_tags = {}
        self.maximum_pose_age_sec = float(maximum_pose_age_sec)
        if self.maximum_pose_age_sec <= 0.0:
            raise ValueError("Maximum pose age must be positive")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            node,
        )
        self._localization_subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            localization_topic,
            self._receive_localization_pose,
            LOCALIZATION_POSE_QOS,
        )
        self._tag_subscription = node.create_subscription(
            TagElementArray,
            visible_tag_topic,
            self._receive_visible_tags,
            10,
        )
        self._active_map_changed = active_map_changed
        self._active_map_subscription = node.create_subscription(
            String,
            "/active_map",
            self._receive_active_map,
            LATCHED_QOS,
        )

    def current_pose(self):
        """Return the latest localized robot pose in map frame."""
        with self._lock:
            source = deepcopy(self._localization_pose)
        if source is None:
            return None
        stamp = source.header.stamp
        stamp_nanoseconds = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )
        if stamp_nanoseconds <= 0:
            return None
        age_nanoseconds = (
            self.node.get_clock().now().nanoseconds - stamp_nanoseconds
        )
        if age_nanoseconds < 0 or age_nanoseconds > (
            self.maximum_pose_age_sec * 1_000_000_000
        ):
            return None
        pose = PoseStamped()
        pose.header = source.header
        pose.pose = source.pose.pose
        return pose

    def visible_tag_pose(self, tag_id: int):
        """Return one currently visible tag transformed into map frame."""
        with self._lock:
            tag = deepcopy(self._visible_tags.get(tag_id))
        if tag is None:
            return None
        source = PoseStamped()
        source.header = tag.pose.header
        source.pose = tag.pose.pose
        try:
            return self._tf_buffer.transform(
                source,
                "map",
                timeout=Duration(seconds=0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            return None

    def _receive_localization_pose(self, message) -> None:
        with self._lock:
            self._localization_pose = deepcopy(message)

    def _receive_visible_tags(self, message) -> None:
        with self._lock:
            self._visible_tags = {
                element.id: deepcopy(element)
                for element in message.elements
            }

    def _receive_active_map(self, message) -> None:
        if self._active_map_changed is not None:
            self._active_map_changed(message.data)


__all__ = ["NavigationSetupStateSource"]
