"""Supervise landmark-assisted recovery of the existing RTAB-Map localizer."""

from dataclasses import dataclass, fields
import math
import time

import numpy as np
import py_trees
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager
from fault_detector_spot.navigation.landmark_observations import LandmarkPoseWindow
from fault_detector_spot.navigation.localization_quality import (
    LocalizationQualityConfig, accepted_localization, localization_quality_error,
)
from fault_detector_spot.shared.geometry.transforms import (
    matrix_to_pose, pose_data_to_pose, pose_to_matrix, pose_to_pose_data,
)
from fault_detector_spot.shared.persistence.basic_settings import parameter_override
from fault_detector_spot.shared.ros.qos_profiles import INITIALPOSE_QOS
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


@dataclass(frozen=True)
class LandmarkRecoveryConfig:
    enabled: bool = True
    required_samples: int = 5
    maximum_sample_span_sec: float = 6.0
    maximum_position_spread_m: float = 0.20
    maximum_rotation_spread_rad: float = math.radians(10.0)
    maximum_tag_age_sec: float = 1.5
    maximum_tag_distance_m: float = 3.0
    maximum_odometry_age_sec: float = 0.25
    stationary_duration_sec: float = 1.0
    stationary_position_tolerance_m: float = 0.03
    stationary_yaw_tolerance_rad: float = math.radians(3.0)
    position_error_threshold_m: float = 0.60
    yaw_error_threshold_rad: float = math.radians(20.0)
    recovery_position_tolerance_m: float = 0.30
    recovery_yaw_tolerance_rad: float = math.radians(10.0)
    settle_time_sec: float = 2.0
    retry_interval_sec: float = 15.0
    maximum_attempts: int = 3
    healthy_samples: int = 3

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("Landmark recovery enabled must be a boolean")
        for name, minimum in (("required_samples", 2), ("maximum_attempts", 1), ("healthy_samples", 2)):
            if type(getattr(self, name)) is not int or getattr(self, name) < minimum:
                raise ValueError(f"Landmark recovery {name} must be an integer >= {minimum}")
        for field in fields(self):
            value = getattr(self, field.name)
            if field.type is float and (
                type(value) not in (int, float) or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"Landmark recovery {field.name} must be positive and finite")
        if self.retry_interval_sec < self.settle_time_sec:
            raise ValueError("Landmark retry interval must allow the settling time")
        if (self.recovery_position_tolerance_m > self.position_error_threshold_m
                or self.recovery_yaw_tolerance_rad > self.yaw_error_threshold_rad):
            raise ValueError("Landmark recovery tolerances must not exceed trigger thresholds")

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(cls):
            key = f"navigation.landmark_recovery.{field.name}"
            if not node.has_parameter(key):
                node.declare_parameter(key, getattr(defaults, field.name))
            values[field.name] = parameter_override(key, node.get_parameter(key).value)
        return cls(**values)


class LandmarkRelocalizer(py_trees.behaviour.Behaviour):
    """Request bounded pose resets; only fresh accepted results confirm recovery."""

    def __init__(self, rtabmap_runtime, tf_buffer, map_repository=None, node=None,
                 name="LandmarkRelocalizer", *,
                 config=None, quality_config=None, monotonic_clock=time.monotonic):
        super().__init__(name)
        self.rtabmap_runtime = rtabmap_runtime
        self.map_repository = map_repository if map_repository is not None else MapRepository(
            rtabmap_runtime.maps_dir,
        )
        self.tf_buffer = tf_buffer
        self.node = node
        self.base_frame = None  # Loaded from the active map's pose convention.
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.config = config
        self.quality_config = quality_config
        self._monotonic_clock = monotonic_clock
        self.blackboard = self.attach_blackboard_client()
        self.initialpose_pub = None
        self.landmark_map = {}
        self.loaded_map_name = None
        self._repository_revision = None
        self._context = None
        self._last_ros_ns = None
        self._reset_recovery(0)

    def setup(self, **kwargs):
        self.node = kwargs.get("node", self.node)
        if self.config is None:
            self.config = LandmarkRecoveryConfig.from_node(self.node)
        if self.quality_config is None:
            self.quality_config = LocalizationQualityConfig.from_node(self.node)
        for key in ("visible_tags", "active_map_name", "last_pose_estimation", "last_localization_info", "command_tree_status"):
            self.blackboard.register_key(key, access=py_trees.common.Access.READ)
        self.blackboard.register_key("localization_reset_hold_until", access=py_trees.common.Access.WRITE)
        self.blackboard.set("localization_reset_hold_until", 0.0)
        self.initialpose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", INITIALPOSE_QOS,
        )

    def shutdown(self):
        if self.initialpose_pub is not None:
            self.node.destroy_publisher(self.initialpose_pub)
            self.initialpose_pub = None
            self.blackboard.set("localization_reset_hold_until", 0.0)

    def _reset_recovery(self, boundary_ns):
        self._minimum_stamp_ns = boundary_ns
        self._windows = {}
        self._window_origins = {}
        self._tag_progress = {}
        self.attempts = 0
        self._last_request_ns = 0
        self._last_request_at = None
        self._healthy_count = 0
        self._last_health_stamp_ns = 0
        self._last_health_at = None
        self._localization_progress = (0, None, None)
        self._odometry_progress = (0, None)
        self._stationary_reference = None
        self._stationary = False
        self._failed = False

    def _load_landmarks_for_map(self, map_name):
        self.landmark_map = {}
        try:
            definition = self.map_repository.load(map_name)
            self.base_frame = definition.localization_frame
            if self.rtabmap_runtime.localization_frame != self.base_frame:
                raise ValueError("Map pose reference differs from the running localization process")
            self.landmark_map = {
                landmark.reference_tag.tag_id: pose_data_to_pose(landmark.pose_map)
                for landmark in definition.localization_landmarks
                if landmark.reference_tag.tag_family == "36h11"
            }
        except (FileNotFoundError, OSError, TypeError, ValueError) as error:
            self.feedback_message = f"Failed to load landmarks: {error}"
            return False
        self.feedback_message = (
            f"Loaded {len(self.landmark_map)} localization landmarks"
            if self.landmark_map else "No localization landmarks found"
        )
        return True

    def _refresh_landmarks(self, map_name, now_ns):
        try:
            revision = self.map_repository.revision(map_name)
        except (OSError, TypeError, ValueError) as error:
            self.landmark_map = {}
            self._repository_revision = None
            self._windows.clear()
            self.feedback_message = f"Landmark metadata unavailable: {error}"
            return False
        if self.loaded_map_name != map_name or revision != self._repository_revision:
            previous = self.landmark_map
            previous_frame = self.base_frame
            if not self._load_landmarks_for_map(map_name):
                self._repository_revision = None
                self._windows.clear()
                return False
            self.loaded_map_name = map_name
            self._repository_revision = revision
            if previous != self.landmark_map or previous_frame != self.base_frame:
                self._reset_recovery(now_ns)
        return True

    @staticmethod
    def _stamp_ns(stamp):
        if not 0 <= stamp.nanosec < 1_000_000_000 or stamp.sec < 0:
            raise ValueError("Invalid timestamp")
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _read(self, key):
        return self.blackboard.get(key) if self.blackboard.exists(key) else None

    def _lookup(self, target, source, stamp):
        transform = self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration())
        if transform.header.frame_id != target or transform.child_frame_id != source:
            raise ValueError(f"Expected TF {target} <- {source}")
        if stamp.nanoseconds and self._stamp_ns(transform.header.stamp) != stamp.nanoseconds:
            raise ValueError("TF does not match the requested acquisition timestamp")
        return pose_to_matrix(pose_data_to_pose(transform_to_pose_data(transform))), transform.header.stamp

    def _observe_odometry(self, now_ns, steady_now):
        """Require measured stationarity; RTAB-Map applies seeds at its next scan."""
        try:
            matrix, stamp = self._lookup(self.odom_frame, self.base_frame, Time())
            stamp_ns = self._stamp_ns(stamp)
            age = (now_ns - stamp_ns) / 1e9
            if stamp_ns <= self._minimum_stamp_ns or not 0 <= age <= self.config.maximum_odometry_age_sec:
                raise ValueError("Current odometry is stale or future-dated")
            previous_stamp, received_at = self._odometry_progress
            if stamp_ns > previous_stamp:
                received_at = steady_now
                self._odometry_progress = (stamp_ns, received_at)
            if (stamp_ns < previous_stamp or received_at is None
                    or steady_now - received_at > self.config.maximum_odometry_age_sec):
                raise ValueError("No advancing odometry")
            if self._stationary_reference is not None:
                reference, _ = self._stationary_reference
                _, yaw = self._pose_error(matrix, reference)
                if (np.linalg.norm(matrix[:3, 3] - reference[:3, 3]) > self.config.stationary_position_tolerance_m
                        or yaw > self.config.stationary_yaw_tolerance_rad):
                    self._stationary_reference = None
            if self._stationary_reference is None:
                self._stationary_reference = (matrix, stamp_ns)
            self._stationary = (
                stamp_ns - self._stationary_reference[1] >= self.config.stationary_duration_sec * 1e9
            )
            return matrix, stamp
        except (tf2_ros.TransformException, TypeError, ValueError):
            self._stationary_reference = None
            self._stationary = False
            return None, None

    def _candidates(self, now_ns, steady_now):
        visible = self._read("visible_tags") or {}
        candidates = []
        active_ids = set()
        boundary = max(self._minimum_stamp_ns, self._last_request_ns)
        for tag_id, tag in visible.items():
            if tag_id not in self.landmark_map:
                continue
            try:
                stamp_ns = self._stamp_ns(tag.pose.header.stamp)
                frame = tag.pose.header.frame_id
                age = (now_ns - stamp_ns) / 1e9
                if stamp_ns <= boundary or not 0 <= age <= self.config.maximum_tag_age_sec:
                    continue
                if not frame or frame in (self.map_frame, self.odom_frame):
                    continue  # Recovery requires a robot-relative observation.
                progress = self._tag_progress.get(tag_id)
                if progress is None or progress[:2] != (stamp_ns, frame):
                    progress = (stamp_ns, frame, steady_now)
                    self._tag_progress[tag_id] = progress
                if steady_now - progress[2] > self.config.maximum_tag_age_sec:
                    continue
                observation = pose_to_matrix(tag.pose.pose)
                distance = float(np.linalg.norm(observation[:3, 3]))
                if distance > self.config.maximum_tag_distance_m:
                    continue
                odom_to_sensor, _ = self._lookup(self.odom_frame, frame, Time.from_msg(tag.pose.header.stamp))
                # A fixed map<-odom correction is comparable across captures
                # even while the robot moves. Never average historical base poses.
                map_to_odom = pose_to_matrix(self.landmark_map[tag_id]) @ np.linalg.inv(
                    odom_to_sensor @ observation,
                )
                window = self._windows.setdefault(tag_id, LandmarkPoseWindow(
                    self.config.required_samples, self.config.maximum_sample_span_sec,
                    self.config.maximum_position_spread_m, self.config.maximum_rotation_spread_rad,
                ))
                if window.sample_count == 0:
                    self._window_origins[tag_id] = odom_to_sensor
                origin = self._window_origins[tag_id]
                active_ids.add(tag_id)
                # Express corrections at a nearby, fixed reference. Comparing
                # map<-odom translations at a distant origin amplifies yaw noise.
                stable = window.add(stamp_ns, pose_to_pose_data(matrix_to_pose(map_to_odom @ origin)), frame)
                if stable is not None:
                    candidates.append((distance, tag_id, pose_to_matrix(pose_data_to_pose(stable)) @ np.linalg.inv(origin)))
            except (tf2_ros.TransformException, TypeError, ValueError, np.linalg.LinAlgError):
                continue
        self._windows = {key: value for key, value in self._windows.items() if key in active_ids}
        self._window_origins = {key: value for key, value in self._window_origins.items() if key in active_ids}
        return sorted(candidates, key=lambda item: (item[0], item[1]))

    def _quality_error(self, pose, now_ns, steady_now):
        if pose is None:
            return localization_quality_error(pose, now_ns, self.quality_config)
        try:
            stamp_ns = self._stamp_ns(pose.header.stamp)
        except (AttributeError, TypeError, ValueError):
            return "Invalid localization timestamp"
        previous_stamp, received_at, received_ros_ns = self._localization_progress
        if stamp_ns > previous_stamp and stamp_ns <= now_ns:
            received_at = steady_now
            received_ros_ns = now_ns
            self._localization_progress = (stamp_ns, received_at, received_ros_ns)
        if stamp_ns <= self._minimum_stamp_ns:
            return "Waiting for localization acquired after the map/runtime change"
        if stamp_ns < previous_stamp:
            return "Localization timestamp regressed"
        if (received_at is None or steady_now - received_at > self.quality_config.maximum_age_sec
                or received_ros_ns is None
                or not 0 <= now_ns - received_ros_ns <= self.quality_config.maximum_age_sec * 1e9):
            return "No advancing localization estimates"
        # Preserve acquisition stamps for TF and registration matching. Only
        # freshness uses receipt time after validating arrival delay once.
        return localization_quality_error(pose, received_ros_ns, self.quality_config)

    @staticmethod
    def _pose_error(first, second):
        difference = first[:2, 3] - second[:2, 3]
        yaw = math.atan2(first[1, 0], first[0, 0]) - math.atan2(second[1, 0], second[0, 0])
        return float(np.linalg.norm(difference)), abs(math.atan2(math.sin(yaw), math.cos(yaw)))

    def _registration_matches(self, pose):
        info = self._read("last_localization_info")
        return (info is not None and info.header.frame_id == self.map_frame
                and info.header.stamp == pose.header.stamp and accepted_localization(info))

    def update(self):
        self._update_recovery()
        # Supervision itself never fails or blocks the command tree. The command
        # manager owns a brief normal-dispatch pause after a reset is published.
        return py_trees.common.Status.SUCCESS

    def _update_recovery(self):
        now_ns = self.node.get_clock().now().nanoseconds
        steady_now = self._monotonic_clock()
        map_name = self._read("active_map_name")
        try:
            mode = self.rtabmap_runtime.get_running_mode()
        except RuntimeError as exception:
            # The process and mode are updated separately by the runtime worker.
            self._context = None
            self._reset_recovery(now_ns)
            self.feedback_message = f"Waiting for localization runtime: {exception}"
            return
        if not self.config.enabled or mode != RtabmapRuntimeManager.MODE_LOCALIZATION or not map_name:
            self._context = None
            self._reset_recovery(now_ns)
            self.feedback_message = "Landmark recovery inactive (requires localization mode and active map)"
            return
        context = (map_name, id(self.rtabmap_runtime.process))
        if context != self._context or (self._last_ros_ns is not None and now_ns < self._last_ros_ns):
            self._reset_recovery(now_ns)
            self._context = context
            self._repository_revision = None
        self._last_ros_ns = now_ns
        if not self._refresh_landmarks(map_name, now_ns):
            return
        if not self.landmark_map:
            self.feedback_message = "No landmarks for active map"
            return
        pose = self._read("last_pose_estimation")
        quality_error = self._quality_error(pose, now_ns, steady_now)
        odom_now, odom_stamp = self._observe_odometry(now_ns, steady_now)
        elapsed = float("inf") if self._last_request_at is None else steady_now - self._last_request_at
        exhausted = self.attempts >= self.config.maximum_attempts and elapsed >= self.config.retry_interval_sec
        failure_detail = (
            f"Landmark recovery failed after {self.attempts} attempts; automatic corrections paused "
            "until localization recovers or the map/runtime changes"
        )
        if exhausted and not self._failed:
            self.node.get_logger().warning(failure_detail)
            self._failed = True
        candidates = self._candidates(now_ns, steady_now)
        if not candidates:
            self._healthy_count = 0
            self.feedback_message = failure_detail if exhausted else "Waiting for fresh, distinct, stable landmark observations"
            return
        _, tag_id, map_to_odom = candidates[0]
        for _, other_id, other in candidates[1:]:
            reference = odom_now if odom_now is not None else self._window_origins[tag_id]
            position, yaw = self._pose_error(map_to_odom @ reference, other @ reference)
            if position > self.config.position_error_threshold_m or yaw > self.config.yaw_error_threshold_rad:
                self._healthy_count = 0
                self.feedback_message = f"Conflicting landmarks {tag_id} and {other_id}; recovery withheld"
                return
        error = None
        if pose is not None:
            try:
                odom_at_estimate, _ = self._lookup(self.odom_frame, self.base_frame, Time.from_msg(pose.header.stamp))
                error = self._pose_error(map_to_odom @ odom_at_estimate, pose_to_matrix(pose.pose.pose))
            except (tf2_ros.TransformException, TypeError, ValueError):
                pass
        agrees = error is not None and (
            error[0] <= self.config.recovery_position_tolerance_m
            and error[1] <= self.config.recovery_yaw_tolerance_rad
        )
        if not quality_error and agrees:
            stamp_ns = self._stamp_ns(pose.header.stamp)
            info = self._read("last_localization_info")
            if info is None or info.header.stamp != pose.header.stamp:
                # Pose and Info callbacks may interleave. Preserve confirmation
                # progress briefly, but absent evidence must not stall recovery.
                recent_confirmation = (
                    self._healthy_count > 0 and self._last_health_at is not None
                    and steady_now - self._last_health_at < self.quality_config.maximum_age_sec
                )
                if not self.attempts or elapsed < self.config.retry_interval_sec or recent_confirmation:
                    self.feedback_message = "Waiting for registration evidence for the fresh localization estimate"
                    return
            if not self._registration_matches(pose):
                self._healthy_count = 0
            elif stamp_ns > self._last_request_ns and elapsed >= self.config.settle_time_sec:
                if stamp_ns > self._last_health_stamp_ns:
                    self._healthy_count += 1
                    self._last_health_stamp_ns = stamp_ns
                    self._last_health_at = steady_now
                if self._healthy_count >= self.config.healthy_samples:
                    if self.attempts:
                        self.node.get_logger().info(f"Landmark recovery confirmed from tag {tag_id}")
                    self.attempts = 0
                    self._failed = False
                    self.feedback_message = f"Localization healthy and agrees with landmark {tag_id}"
                    return
                self.feedback_message = f"Confirming localization recovery ({self._healthy_count}/{self.config.healthy_samples})"
                return
        self._healthy_count = 0
        disagrees = error is not None and (
            error[0] > self.config.position_error_threshold_m
            or error[1] > self.config.yaw_error_threshold_rad
        )
        if not quality_error and not disagrees and not self.attempts:
            self.feedback_message = "Localization within landmark correction thresholds"
            return
        if elapsed < self.config.retry_interval_sec:
            self.feedback_message = "Waiting for post-correction localization results"
            return
        if exhausted:
            self.feedback_message = failure_detail
            return
        command_status = self._read("command_tree_status")
        if command_status is None or command_status == py_trees.common.Status.RUNNING:
            self._stationary_reference = None
            self._stationary = False
            self.feedback_message = "Landmark recovery pending; waiting for the running command to finish"
            return
        if odom_now is None or not self._stationary:
            self.feedback_message = "Landmark recovery pending; waiting for fresh, stationary odometry"
            return
        try:
            corrected = matrix_to_pose(map_to_odom @ odom_now)
            # Catch atomic edits made during this tick before using a removed anchor.
            if self.map_repository.revision(map_name) != self._repository_revision:
                self.feedback_message = "Landmarks changed; waiting for new observations"
                return
        except (tf2_ros.TransformException, OSError, TypeError, ValueError) as exception:
            self.feedback_message = f"Landmark correction unavailable: {exception}"
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = odom_stamp
        message.pose.pose = corrected
        for index in (0, 7, 14):
            message.pose.covariance[index] = self.config.maximum_position_spread_m ** 2
        for index in (21, 28, 35):
            message.pose.covariance[index] = self.config.maximum_rotation_spread_rad ** 2
        # RTAB-Map consumes the pose as a seed, not a covariance-weighted update.
        self.initialpose_pub.publish(message)
        # Keep newly queued motion from starting during nominal scan processing.
        # This bounded pause is not an acknowledgement of seed consumption. Do
        # not shorten it on metadata/clock resets: the seed is already in flight.
        self.blackboard.set(
            "localization_reset_hold_until", steady_now + self.config.settle_time_sec,
        )
        self.attempts += 1
        self._last_request_ns = now_ns
        self._last_request_at = steady_now
        self._healthy_count = 0
        self._last_health_stamp_ns = 0
        self._last_health_at = None
        self._windows.clear()
        self.feedback_message = (
            f"Landmark {tag_id} recovery requested ({self.attempts}/{self.config.maximum_attempts}); "
            "waiting for confirmed localization"
        )
        self.node.get_logger().info(self.feedback_message)
