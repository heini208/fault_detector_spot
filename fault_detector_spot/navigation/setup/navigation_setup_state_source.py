"""Collect runtime pose and tag inputs for navigation authoring."""

from copy import deepcopy
from dataclasses import dataclass
import math
from threading import RLock
import time

import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms with tf2
import tf2_ros
from fault_detector_msgs.msg import TagElementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.time import Time
from fault_detector_spot.navigation.landmark_observations import LandmarkPoseWindow
from fault_detector_spot.navigation.localization_quality import (
    LocalizationQualityConfig,
    localization_quality_error,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from fault_detector_spot.shared.persistence.basic_settings import parameter_override
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data
from std_msgs.msg import String

from fault_detector_spot.shared.ros.qos_profiles import (
    LATCHED_QOS,
    LOCALIZATION_POSE_QOS,
)


@dataclass(frozen=True)
class LandmarkSaveConfig:
    """Limits for saving a stationary tag in the active map."""

    required_samples: int = 3
    maximum_sample_span_sec: float = 4.0
    maximum_position_spread_m: float = 0.05
    maximum_rotation_spread_rad: float = math.radians(5.0)
    maximum_observation_age_sec: float = 1.5

    def __post_init__(self):
        self.new_window()
        if (not math.isfinite(self.maximum_observation_age_sec)
                or self.maximum_observation_age_sec <= 0.0):
            raise ValueError("Landmark observation age must be positive and finite")

    def new_window(self):
        return LandmarkPoseWindow(
            self.required_samples, self.maximum_sample_span_sec,
            self.maximum_position_spread_m, self.maximum_rotation_spread_rad,
        )

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for name in cls.__dataclass_fields__:
            key = f"navigation.landmark_save.{name}"
            if not node.has_parameter(key):
                node.declare_parameter(key, getattr(defaults, name))
            values[name] = parameter_override(key, node.get_parameter(key).value)
        return cls(**values)


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
        self._landmark_localization = None
        self._landmark_healthy_since_ns = None
        self._landmark_health_generation = 0
        self._landmark_save_config = None
        self._localization_quality_config = None
        self._landmark_windows = {}
        self._qualified_landmarks = {}
        self._landmark_last_observations = {}
        self._landmark_errors = {}
        self._visible_tags_received_at = None
        self._tag_generation = 0
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
        self._landmark_localization = None
        self._visible_tags.clear()
        self._visible_tags_received_at = None
        self._reset_landmark_health()

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
        """Return a qualified full tag pose, or explain why saving is unsafe."""
        with self._lock:
            self._require_landmark_localization()
            tag = self._visible_tags.get(tag_id)
            if tag is None:
                raise ValueError(f"Tag {tag_id} is not currently visible")
            self._require_fresh_tag(tag)
            received_at = self._visible_tags_received_at
            latest = self._landmark_last_observations.get(tag_id)
            maximum_age = self._landmark_save_config.maximum_observation_age_sec
            if (received_at is None
                    or not 0 <= self._monotonic_clock() - received_at <= maximum_age
                    or (latest is not None
                        and not 0 <= self._monotonic_clock() - latest[2] <= maximum_age)):
                self._clear_landmark_tag(tag_id, "No fresh distinct tag observation received")
            pose = self._qualified_landmarks.get(tag_id)
            if pose is None:
                raise ValueError(self._landmark_errors.get(
                    tag_id, "Wait for distinct stable landmark observations",
                ))
            observation = (
                self._stamp_ns(tag.pose.header.stamp), tag.pose.header.frame_id,
            )
            if latest is None or latest[:2] != observation:
                raise ValueError("Wait for the latest tag observation to be qualified")
            return deepcopy(pose)

    def _reset_landmark_health(self):
        self._landmark_healthy_since_ns = None
        self._landmark_health_generation += 1
        self._clear_landmark_samples()

    def _clear_landmark_samples(self):
        self._landmark_windows.clear()
        self._qualified_landmarks.clear()
        self._landmark_last_observations.clear()
        self._landmark_errors.clear()

    def _clear_landmark_tag(self, tag_id, detail):
        self._landmark_windows.pop(tag_id, None)
        self._qualified_landmarks.pop(tag_id, None)
        self._landmark_last_observations.pop(tag_id, None)
        self._landmark_errors[tag_id] = detail

    def _require_landmark_localization(self):
        if self._landmark_save_config is None:
            self._landmark_save_config = LandmarkSaveConfig.from_node(self.node)
        if self._localization_quality_config is None:
            self._localization_quality_config = LocalizationQualityConfig.from_node(self.node)
        try:
            self._require_live_localization()
            if (self._monotonic_clock() - self._localization_received_at
                    > self._localization_quality_config.maximum_age_sec):
                raise ValueError("Cannot save landmark: localization estimate is stale")
            if self._landmark_localization is None:
                raise ValueError(self._localization_error or "Wait for a valid localization estimate")
            error = localization_quality_error(
                self._landmark_localization,
                self.node.get_clock().now().nanoseconds,
                self._localization_quality_config,
            )
            if error:
                raise ValueError(f"Cannot save landmark: {error}")
        except ValueError:
            self._reset_landmark_health()
            raise
        if self._landmark_healthy_since_ns is None:
            self._landmark_healthy_since_ns = self._stamp_ns(
                self._landmark_localization.header.stamp,
            )

    def _require_fresh_tag(self, tag):
        stamp_ns = self._stamp_ns(tag.pose.header.stamp)
        now_ns = self.node.get_clock().now().nanoseconds
        if not 0 <= tag.pose.header.stamp.nanosec < 1_000_000_000:
            raise ValueError("Tag observation timestamp is invalid")
        if not tag.pose.header.frame_id.strip():
            raise ValueError("Tag observation has no source frame")
        if stamp_ns <= 0 or now_ns <= 0 or stamp_ns > now_ns:
            raise ValueError("Tag observation has an invalid or future timestamp")
        if stamp_ns < self._minimum_pose_stamp_ns:
            raise ValueError("Wait for a tag observation acquired after the map change")
        if (now_ns - stamp_ns) / 1e9 > self._landmark_save_config.maximum_observation_age_sec:
            raise ValueError("Tag observation is stale; wait for a new camera observation")
        if (self._landmark_healthy_since_ns is None
                or stamp_ns < self._landmark_healthy_since_ns):
            raise ValueError("Wait for a tag observation acquired after trustworthy localization resumed")
        pose_to_pose_data(tag.pose)

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
                self._landmark_localization = None
                self._reset_landmark_health()
                return
            received_at = self._monotonic_clock()
            if self._localization_quality_config is not None:
                # Detect gaps even when no tag/save callback ran while the
                # previous estimate was stale. Delayed images from that gap
                # must not be transformed through the old map localization.
                previous_error = localization_quality_error(
                    self._landmark_localization, now_ns,
                    self._localization_quality_config,
                )
                if (previous_error or self._localization_received_at is None
                        or not 0 <= received_at - self._localization_received_at
                        <= min(self.maximum_pose_age_sec,
                               self._localization_quality_config.maximum_age_sec)):
                    self._reset_landmark_health()
                if localization_quality_error(
                    message, now_ns, self._localization_quality_config,
                ):
                    self._reset_landmark_health()
                elif self._landmark_healthy_since_ns is None:
                    self._landmark_healthy_since_ns = stamp_ns
            # RTAB-Map timestamps the sensor acquisition, before processing.
            # Validate that delay once, then require continuing processed updates.
            self._localization_stamp_ns = stamp_ns
            self._localization_received_at = received_at
            self._localization_error = ""
            self._landmark_localization = deepcopy(message)

    def _receive_visible_tags(self, message) -> None:
        with self._lock:
            self._tag_generation += 1
            tag_generation = self._tag_generation
            generation = self._pose_generation
            self._visible_tags = {
                element.id: deepcopy(element)
                for element in message.elements
            }
            self._visible_tags_received_at = self._monotonic_clock()
            for tag_id in set(self._landmark_windows) - self._visible_tags.keys():
                self._clear_landmark_tag(tag_id, "Tag is no longer visible")
            if not self._visible_tags:
                self._clear_landmark_samples()
                return
            tags = deepcopy(self._visible_tags)
        for tag_id, tag in tags.items():
            with self._lock:
                if generation != self._pose_generation or tag_generation != self._tag_generation:
                    return
                try:
                    self._require_landmark_localization()
                    self._require_fresh_tag(tag)
                except (TypeError, ValueError) as exception:
                    self._clear_landmark_tag(tag_id, str(exception))
                    continue
                stamp_ns = self._stamp_ns(tag.pose.header.stamp)
                frame = tag.pose.header.frame_id
                previous = self._landmark_last_observations.get(tag_id)
                if (previous is not None and not 0 <= self._monotonic_clock() - previous[2]
                        <= self._landmark_save_config.maximum_observation_age_sec):
                    self._clear_landmark_tag(tag_id, "Tag observation stream was interrupted")
                    previous = None
                if previous is not None and previous[:2] == (stamp_ns, frame):
                    continue
                health_generation = self._landmark_health_generation
            try:
                pose = self._tf_buffer.transform(tag.pose, "map", timeout=Duration())
                if (pose.header.frame_id != "map"
                        or self._stamp_ns(pose.header.stamp) != stamp_ns):
                    raise ValueError("Tag transform must use map at the observation timestamp")
                map_pose = pose_to_pose_data(pose)
                with self._lock:
                    if generation != self._pose_generation or tag_generation != self._tag_generation:
                        return
                    if health_generation != self._landmark_health_generation:
                        return
                    self._require_landmark_localization()
                    self._require_fresh_tag(tag)
                    window = self._landmark_windows.setdefault(
                        tag_id, self._landmark_save_config.new_window(),
                    )
                    qualified = window.add(stamp_ns, map_pose, frame)
                    self._landmark_last_observations[tag_id] = (
                        stamp_ns, frame, self._monotonic_clock(),
                    )
                    if qualified is None:
                        self._qualified_landmarks.pop(tag_id, None)
                        self._landmark_errors[tag_id] = window.detail
                    else:
                        self._qualified_landmarks[tag_id] = qualified
                        self._landmark_errors.pop(tag_id, None)
            except (tf2_ros.TransformException, TypeError, ValueError) as exception:
                with self._lock:
                    if generation == self._pose_generation and tag_generation == self._tag_generation:
                        self._clear_landmark_tag(tag_id, f"Cannot save landmark: {exception}")

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


__all__ = ["LandmarkSaveConfig", "NavigationSetupStateSource"]
