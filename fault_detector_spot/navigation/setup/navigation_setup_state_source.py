"""Collect runtime pose and tag inputs for navigation authoring."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, replace
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
from fault_detector_spot.navigation.landmark_observations import (
    LandmarkObservationPending, LandmarkPoseWindow, NavigationObservationPending,
)
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


@dataclass(frozen=True)
class _PoseCapture:
    started_ros_ns: int
    started_at: float
    pose_generation: int


@dataclass(frozen=True)
class _LandmarkCapture(_PoseCapture):
    tag_id: int


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
        self._localization_received_ros_ns = None
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
        self._landmark_pending_observations = {}
        self._landmark_errors = {}
        self._visible_tags_received_at = None
        self._tag_generation = 0
        self._landmark_capture = None
        self._waypoint_capture = None
        self._maximum_capture_localization_delay_sec = None
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
        ros_age_ns = self.node.get_clock().now().nanoseconds - self._localization_received_ros_ns
        if (age_sec < 0 or age_sec > self.maximum_pose_age_sec
                or not 0 <= ros_age_ns <= self.maximum_pose_age_sec * 1e9):
            last_rejection = (
                f"; last rejected estimate: {self._localization_error}"
                if self._localization_error else ""
            )
            raise ValueError(
                "Localization estimate is stale; no new valid estimate received "
                f"within {self.maximum_pose_age_sec:g} seconds{last_rejection}"
            )
        # A slower estimate is usable only inside the explicit capture that
        # requested new data. Finishing a save must not relax the live reader.
        arrival_age = (self._localization_received_ros_ns - self._localization_stamp_ns) / 1e9
        if arrival_age > self._localization_arrival_limit(self._localization_stamp_ns):
            raise ValueError("Localization estimate is stale outside its requested capture")

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _invalidate_localization(self, detail, minimum_stamp_ns=0):
        self._localization_stamp_ns = None
        self._localization_received_at = None
        self._localization_received_ros_ns = None
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
        self.finish_landmark_capture(self._landmark_capture)
        self.finish_waypoint_capture(self._waypoint_capture)
        if self._clock_jump is not None:
            self._clock_jump.unregister()
            self._clock_jump = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None

    def begin_landmark_capture(self, tag_id: int):
        """Start qualifying only new observations for one requested tag."""
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
            raise ValueError("Landmark tag ID must be a non-negative integer")
        with self._lock:
            self._begin_capture_settings()
            if self._landmark_save_config is None:
                self._landmark_save_config = LandmarkSaveConfig.from_node(self.node)
            if self._localization_quality_config is None:
                self._localization_quality_config = LocalizationQualityConfig.from_node(self.node)
            self._tag_generation += 1
            self._reset_landmark_health()
            self._visible_tags.clear()
            self._visible_tags_received_at = None
            capture = _LandmarkCapture(
                self.node.get_clock().now().nanoseconds,
                self._monotonic_clock(), self._pose_generation, tag_id,
            )
            self._landmark_capture = capture
            return capture

    def _begin_capture_settings(self):
        if self._landmark_capture is not None or self._waypoint_capture is not None:
            raise RuntimeError("A navigation save capture is already active")
        if self._maximum_capture_localization_delay_sec is None:
            key = "navigation.authoring.maximum_localization_delay_sec"
            if not self.node.has_parameter(key):
                self.node.declare_parameter(key, 5.0)
            value = parameter_override(key, self.node.get_parameter(key).value)
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError("Navigation authoring localization delay must be positive and finite")
            self._maximum_capture_localization_delay_sec = float(value)
        if self._localization_quality_config is None:
            self._localization_quality_config = LocalizationQualityConfig.from_node(self.node)

    def begin_waypoint_capture(self):
        """Wait for localization acquired after this waypoint Save click."""
        with self._lock:
            self._begin_capture_settings()
            capture = _PoseCapture(
                self.node.get_clock().now().nanoseconds,
                self._monotonic_clock(), self._pose_generation,
            )
            self._waypoint_capture = capture
            return capture

    def finish_waypoint_capture(self, token):
        with self._lock:
            if token is not None and token is self._waypoint_capture:
                self._waypoint_capture = None

    def waypoint_capture_pose(self, token):
        """Read a qualified current robot pose, keeping transient failures pending."""
        with self._lock:
            self._require_waypoint_capture(token)
            try:
                self._require_capture_localization(token)
                # Keep the lock across the shared TF read and its generation
                # checks so a cancelled capture cannot publish a late result.
                pose = self.current_pose()
                self._require_waypoint_capture(token)
                return pose
            except (TypeError, ValueError) as exception:
                # Map/clock/identity changes are terminal, not availability gaps.
                self._require_waypoint_capture(token)
                raise NavigationObservationPending(str(exception)) from exception

    def _require_waypoint_capture(self, token):
        if token is None or token is not self._waypoint_capture:
            raise ValueError("No matching waypoint capture is active")
        self._require_capture_context(token, "waypoint")

    def _require_capture_context(self, capture, kind):
        if capture.pose_generation != self._pose_generation:
            raise ValueError(f"Map or clock changed during {kind} capture; start a new capture")
        if (self._monotonic_clock() < capture.started_at
                or self.node.get_clock().now().nanoseconds < capture.started_ros_ns):
            raise ValueError(f"Clock changed during {kind} capture; start a new capture")

    def _localization_arrival_limit(self, stamp_ns):
        capture = self._landmark_capture or self._waypoint_capture
        if (capture is not None and capture.pose_generation == self._pose_generation
                and stamp_ns > capture.started_ros_ns):
            return self._maximum_capture_localization_delay_sec
        return self.maximum_pose_age_sec

    def _capture_quality_error(self, message, received_ros_ns):
        # Covariance validation is unchanged. Processing delay has its own
        # request-scoped bound; maximum_age_sec still bounds gaps in the stream.
        return localization_quality_error(
            message, received_ros_ns,
            replace(self._localization_quality_config,
                    maximum_age_sec=self._maximum_capture_localization_delay_sec),
        )

    def _require_capture_localization(self, capture):
        if (self._localization_stamp_ns is None
                or self._localization_stamp_ns <= capture.started_ros_ns):
            detail = "Waiting for localization acquired after Save was requested"
            if self._localization_error:
                detail += f"; {self._localization_error}"
            raise ValueError(detail)
        self._require_live_localization()
        if (self._monotonic_clock() - self._localization_received_at
                > self._localization_quality_config.maximum_age_sec
                or not 0 <= self.node.get_clock().now().nanoseconds - self._localization_received_ros_ns
                <= self._localization_quality_config.maximum_age_sec * 1e9):
            raise ValueError("Cannot save: localization estimate is stale")
        if self._landmark_localization is None:
            raise ValueError(self._localization_error or "Wait for a valid localization estimate")
        error = self._capture_quality_error(
            self._landmark_localization, self._localization_received_ros_ns,
        )
        if error:
            raise ValueError(f"Cannot save: {error}")

    def finish_landmark_capture(self, token):
        """Release this capture without cancelling a newer request."""
        with self._lock:
            if token is None or token is not self._landmark_capture:
                return
            self._landmark_capture = None
            self._tag_generation += 1
            self._visible_tags.clear()
            self._visible_tags_received_at = None
            self._reset_landmark_health()

    def _require_landmark_capture(self, tag_id):
        capture = self._landmark_capture
        if capture is None:
            raise ValueError("No landmark capture is active")
        if tag_id != capture.tag_id:
            raise ValueError("Tag does not match the active landmark capture")
        self._require_capture_context(capture, "landmark")
        return capture

    def visible_tag_pose(self, tag_id: int):
        """Read this capture's result, or report temporary data availability."""
        with self._lock:
            self._require_landmark_capture(tag_id)
            try:
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
                            and (not 0 <= self._monotonic_clock() - latest[2] <= maximum_age
                                 or not 0 <= self.node.get_clock().now().nanoseconds - latest[0]
                                 <= maximum_age * 1e9))):
                    self._clear_landmark_tag(tag_id, "No fresh distinct tag observation received")
                pose = self._qualified_landmarks.get(tag_id)
                if pose is None:
                    raise ValueError(self._landmark_errors.get(
                        tag_id, "Wait for distinct stable landmark observations",
                    ))
                # New camera messages may still be waiting for their exact TF.
                return deepcopy(pose)
            except (TypeError, ValueError) as exception:
                raise LandmarkObservationPending(str(exception)) from exception

    def _reset_landmark_health(self):
        self._landmark_healthy_since_ns = None
        self._landmark_health_generation += 1
        self._clear_landmark_samples()

    def _clear_landmark_samples(self):
        self._landmark_windows.clear()
        self._qualified_landmarks.clear()
        self._landmark_last_observations.clear()
        self._landmark_pending_observations.clear()
        self._landmark_errors.clear()

    def _clear_landmark_tag(self, tag_id, detail):
        self._landmark_windows.pop(tag_id, None)
        self._qualified_landmarks.pop(tag_id, None)
        self._landmark_last_observations.pop(tag_id, None)
        self._landmark_pending_observations.pop(tag_id, None)
        self._landmark_errors[tag_id] = detail

    def _require_landmark_localization(self):
        if self._landmark_save_config is None:
            self._landmark_save_config = LandmarkSaveConfig.from_node(self.node)
        if self._localization_quality_config is None:
            self._localization_quality_config = LocalizationQualityConfig.from_node(self.node)
        try:
            capture = self._landmark_capture
            if capture is None:
                raise ValueError("No landmark capture is active")
            self._require_capture_localization(capture)
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
        if (self._landmark_capture is None
                or stamp_ns <= self._landmark_capture.started_ros_ns):
            raise ValueError("Waiting for a tag observation acquired after Save was requested")
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
                if (stamp_ns <= 0 or now_ns <= 0 or age_sec < 0
                        or not 0 <= message.header.stamp.nanosec < 1_000_000_000):
                    raise ValueError("Localization estimate has an invalid or future timestamp")
                arrival_limit = self._localization_arrival_limit(stamp_ns)
                if age_sec > arrival_limit:
                    raise ValueError(
                        f"Localization estimate arrived {age_sec:.2f} seconds old; "
                        f"maximum is {arrival_limit:g} seconds"
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
            if self._landmark_capture is not None and self._localization_quality_config is not None:
                # Validate acquisition delay at receipt, then track the gap
                # between processed updates. Counting RTAB-Map's processing
                # delay again here would reset healthy capture windows each tick.
                previous_error = self._capture_quality_error(
                    self._landmark_localization, self._localization_received_ros_ns,
                )
                if (previous_error or self._localization_received_at is None
                        or not 0 <= received_at - self._localization_received_at
                        <= min(self.maximum_pose_age_sec,
                               self._localization_quality_config.maximum_age_sec)
                        or self._localization_received_ros_ns is None
                        or not 0 <= now_ns - self._localization_received_ros_ns
                        <= self._localization_quality_config.maximum_age_sec * 1e9):
                    self._reset_landmark_health()
                if self._capture_quality_error(message, now_ns):
                    self._reset_landmark_health()
                elif self._landmark_healthy_since_ns is None:
                    self._landmark_healthy_since_ns = stamp_ns
            # RTAB-Map timestamps the sensor acquisition, before processing.
            # Validate that delay once, then require continuing processed updates.
            self._localization_stamp_ns = stamp_ns
            self._localization_received_at = received_at
            self._localization_received_ros_ns = now_ns
            self._localization_error = ""
            self._landmark_localization = deepcopy(message)

    def _receive_visible_tags(self, message) -> None:
        with self._lock:
            self._tag_generation += 1
            tag_generation = self._tag_generation
            generation = self._pose_generation
            capture = self._landmark_capture
            if capture is None or capture.pose_generation != generation:
                return
            self._visible_tags = {
                element.id: deepcopy(element)
                for element in message.elements
                if element.id == capture.tag_id
            }
            self._visible_tags_received_at = self._monotonic_clock()
            tracked = self._landmark_windows.keys() | self._landmark_pending_observations.keys()
            for tag_id in tracked - self._visible_tags.keys():
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
                pending = self._landmark_pending_observations.get(tag_id)
                newest = (
                    (self._stamp_ns(pending[-1][0].pose.header.stamp), pending[-1][0].pose.header.frame_id)
                    if pending else previous
                )
                if newest is not None and (stamp_ns < newest[0] or frame != newest[1]):
                    self._clear_landmark_tag(tag_id, "Tag acquisition changed; collecting new observations")
                    newest = None
                pending = self._landmark_pending_observations.setdefault(tag_id, deque(maxlen=64))
                if newest is None or newest[:2] != (stamp_ns, frame):
                    pending.append((tag, self._monotonic_clock()))
                observations = list(pending)
                health_generation = self._landmark_health_generation
            for observation, received_at in observations:
                with self._lock:
                    if generation != self._pose_generation or tag_generation != self._tag_generation:
                        return
                    if health_generation != self._landmark_health_generation:
                        return
                    maximum_age = self._landmark_save_config.maximum_observation_age_sec
                    stamp_ns = self._stamp_ns(observation.pose.header.stamp)
                    if (not 0 <= self._monotonic_clock() - received_at <= maximum_age
                            or self.node.get_clock().now().nanoseconds - stamp_ns > maximum_age * 1e9):
                        self._discard_pending_through(tag_id, stamp_ns, keep_latest=True)
                        continue
                try:
                    pose = self._tf_buffer.transform(observation.pose, "map", timeout=Duration())
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
                        self._require_fresh_tag(observation)
                        window = self._landmark_windows.setdefault(
                            tag_id, self._landmark_save_config.new_window(),
                        )
                        qualified = window.add(stamp_ns, map_pose, observation.pose.header.frame_id)
                        self._landmark_last_observations[tag_id] = (
                            stamp_ns, observation.pose.header.frame_id, received_at,
                        )
                        self._discard_pending_through(tag_id, stamp_ns)
                        if qualified is None:
                            self._qualified_landmarks.pop(tag_id, None)
                            self._landmark_errors[tag_id] = window.detail
                        else:
                            self._qualified_landmarks[tag_id] = qualified
                            self._landmark_errors.pop(tag_id, None)
                except tf2_ros.TransformException as exception:
                    with self._lock:
                        if generation != self._pose_generation or tag_generation != self._tag_generation:
                            return
                        # A temporarily missing TF says nothing about stability.
                        # Retain both good samples and the capture for a later
                        # snapshot, bounded by its original acquisition/receipt.
                        window = self._landmark_windows.get(tag_id)
                        count = window.sample_count if window is not None else 0
                        self._landmark_errors[tag_id] = (
                            f"Waiting for tag transform ({count}/{self._landmark_save_config.required_samples} "
                            f"stable observations retained): {exception}"
                        )
                except (TypeError, ValueError) as exception:
                    with self._lock:
                        if generation == self._pose_generation and tag_generation == self._tag_generation:
                            self._clear_landmark_tag(tag_id, f"Cannot save landmark: {exception}")
                    break

    def _discard_pending_through(self, tag_id, stamp_ns, *, keep_latest=False):
        pending = self._landmark_pending_observations.get(tag_id)
        if pending is not None:
            # Keep an expired latest capture as a timestamp/receipt marker, so
            # republishing it cannot restart its waiting period on a paused clock.
            while (len(pending) > int(keep_latest)
                   and self._stamp_ns(pending[0][0].pose.header.stamp) <= stamp_ns):
                pending.popleft()

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


__all__ = ["LandmarkObservationPending", "LandmarkSaveConfig", "NavigationSetupStateSource"]
