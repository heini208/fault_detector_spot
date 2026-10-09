"""Collect runtime pose and tag inputs for navigation authoring."""

from copy import deepcopy
import math
from threading import RLock
import time

import tf2_ros
from fault_detector_msgs.msg import TagElementArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.time import Time
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data
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
        monotonic_clock=time.monotonic,
    ):
        self.node = node
        self._lock = RLock()
        self._localization_stamp_ns = None
        self._localization_received_at = None
        self._localization_error = "No localization estimate received yet; wait for RTAB-Map"
        self._minimum_pose_stamp_ns = 0
        self._pose_generation = 0
        self._active_map = None
        self._monotonic_clock = monotonic_clock
        self._visible_tags = {}
        self.maximum_pose_age_sec = float(maximum_pose_age_sec)
        if (not math.isfinite(self.maximum_pose_age_sec)
                or self.maximum_pose_age_sec <= 0.0):
            raise ValueError("Maximum pose age must be positive and finite")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            node,
        )
        self._clock_jump = node.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._receive_clock_jump,
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
        """Return the current robot TF while localization updates remain live."""
        with self._lock:
            self._require_live_localization()
            generation = self._pose_generation
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(),
            )
        except tf2_ros.TransformException as exception:
            raise ValueError(
                "Current robot transform map <- base_link is unavailable; "
                "wait for localization and odometry TF"
            ) from exception
        with self._lock:
            if generation != self._pose_generation:
                raise ValueError(
                    "Map or clock changed while reading the robot pose; retry after localization"
                )
            self._require_live_localization()
            if (transform.header.frame_id != "map"
                    or transform.child_frame_id != "base_link"):
                raise ValueError("Current robot transform must be map <- base_link")
            stamp_ns = self._stamp_ns(transform.header.stamp)
            age_sec = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if stamp_ns <= 0 or age_sec < 0:
                raise ValueError("Current robot transform has an invalid or future timestamp")
            if (age_sec > self.maximum_pose_age_sec
                    or stamp_ns < self._minimum_pose_stamp_ns
                    or stamp_ns < self._localization_stamp_ns):
                raise ValueError(
                    "Current robot transform is stale; wait for localization and odometry TF"
                )
            try:
                return transform_to_pose_data(transform)
            except (TypeError, ValueError) as exception:
                raise ValueError(f"Current robot transform is invalid: {exception}") from exception

    def _require_live_localization(self):
        if self._localization_received_at is None:
            raise ValueError(self._localization_error)
        age_sec = self._monotonic_clock() - self._localization_received_at
        if age_sec < 0 or age_sec > self.maximum_pose_age_sec:
            raise ValueError(
                "Localization estimate is stale; no new valid estimate received "
                f"within {self.maximum_pose_age_sec:g} seconds"
            )

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _invalidate_localization(self, detail, minimum_stamp_ns=0):
        self._localization_stamp_ns = None
        self._localization_received_at = None
        self._localization_error = detail
        self._minimum_pose_stamp_ns = minimum_stamp_ns
        self._pose_generation += 1

    def _receive_clock_jump(self, _jump):
        with self._lock:
            self._invalidate_localization(
                "Clock changed; wait for a new localization estimate",
            )
            # Humble's buffer does not reset on ROS clock jumps. Retaining its
            # future dynamic samples would reject the new timeline's TF updates.
            self._tf_buffer.clear()

    def close(self):
        """Release the clock callback and TF listener with the API node."""
        if self._clock_jump is not None:
            self._clock_jump.unregister()
            self._clock_jump = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None

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
            pose = self._tf_buffer.transform(
                source,
                "map",
                timeout=Duration(seconds=0.2),
            )
            if pose.header.frame_id != "map":
                return None
            return pose_to_pose_data(pose)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            return None

    def _receive_localization_pose(self, message) -> None:
        with self._lock:
            stamp_ns = self._stamp_ns(message.header.stamp)
            now_ns = self.node.get_clock().now().nanoseconds
            age_sec = (now_ns - stamp_ns) / 1e9
            try:
                if message.header.frame_id != "map":
                    raise ValueError("Localization estimate must use the map frame")
                if stamp_ns <= 0 or now_ns <= 0 or age_sec < 0:
                    raise ValueError("Localization estimate has an invalid or future timestamp")
                if age_sec > self.maximum_pose_age_sec:
                    raise ValueError(
                        f"Localization estimate arrived {age_sec:.2f} seconds old; "
                        f"maximum is {self.maximum_pose_age_sec:g} seconds"
                    )
                if stamp_ns < self._minimum_pose_stamp_ns:
                    raise ValueError("Wait for a localization estimate acquired after the map change")
                if (self._localization_stamp_ns is not None
                        and stamp_ns <= self._localization_stamp_ns):
                    return
                pose_to_pose_data(message.pose.pose)
            except (TypeError, ValueError) as exception:
                self._localization_error = str(exception)
                return
            # RTAB-Map timestamps the sensor acquisition, before processing.
            # Validate that delay once, then require continuing processed updates.
            self._localization_stamp_ns = stamp_ns
            self._localization_received_at = self._monotonic_clock()
            self._localization_error = ""

    def _receive_visible_tags(self, message) -> None:
        with self._lock:
            self._visible_tags = {
                element.id: deepcopy(element)
                for element in message.elements
            }

    def _receive_active_map(self, message) -> None:
        with self._lock:
            map_name = message.data.strip()
            if map_name != self._active_map:
                self._active_map = map_name
                self._invalidate_localization(
                    "Map changed; wait for a new localization estimate",
                    minimum_stamp_ns=self.node.get_clock().now().nanoseconds,
                )
        if self._active_map_changed is not None:
            self._active_map_changed(message.data)


__all__ = ["NavigationSetupStateSource"]
