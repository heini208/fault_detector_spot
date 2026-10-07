"""Observe fresh filtered depth and stationary base state for local planning."""

import math
from threading import RLock
import time

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from fault_detector_spot.shared.runtime_source import RuntimeSource


class MoveItEnvironmentSource(RuntimeSource):
    """Observe one updater's output; never change the scene or move the robot."""

    def __init__(self, node, config, monotonic_clock=time.monotonic):
        self.node = node
        self._clock = monotonic_clock
        self._lock = RLock()
        self._max_cloud_age = config.get("environment.max_cloud_age_sec")
        self._max_state_age = config.get("environment.max_state_age_sec")
        self._linear_limit = config.get("environment.stationary_linear_mps")
        self._angular_limit = config.get("environment.stationary_angular_rad_s")
        self._translation_limit = config.get("environment.max_body_translation_m")
        self._rotation_limit = config.get("environment.max_body_rotation_rad")
        self._settle_time = config.get("environment.settle_time_sec")
        for value in (self._max_cloud_age, self._max_state_age,
                      self._linear_limit, self._angular_limit,
                      self._translation_limit, self._rotation_limit, self._settle_time):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Environment observation limits must be positive")
        self._odometry = None
        self._state_received = None
        self._cloud_stamp = None
        self._cloud_received = None
        self._last_motion = -math.inf
        self._began_at = None
        self._cloud_after_ros = None
        self._anchor = None
        self._subscriptions = [
            node.create_subscription(
                Odometry, "/odometry", self._receive_odometry,
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                PointCloud2, config.get("environment.filtered_cloud_topic"),
                self._receive_cloud, qos_profile_sensor_data,
            ),
        ]

    def _ros_now(self):
        return self.node.get_clock().now().nanoseconds

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def _receive_odometry(self, message):
        twist = message.twist.twist
        velocity = [getattr(twist.linear, axis) for axis in "xyz"]
        angular = [getattr(twist.angular, axis) for axis in "xyz"]
        with self._lock:
            self._odometry = message
            self._state_received = self._clock()
            if (not all(math.isfinite(value) for value in velocity + angular)
                    or math.hypot(*velocity) > self._linear_limit
                    or math.hypot(*angular) > self._angular_limit):
                self._last_motion = self._state_received

    def _receive_cloud(self, message):
        # The updater publishes only points outside its robot mask. A nonempty
        # message alone is insufficient: organized clouds may contain only NaNs.
        fields = {field.name: field for field in message.fields}
        if not message.width or not message.height or any(
            axis not in fields or fields[axis].datatype != 7 for axis in "xyz"
        ):
            return
        try:
            dtype = np.dtype({
                "names": list("xyz"),
                "formats": [(">" if message.is_bigendian else "<") + "f4"] * 3,
                "offsets": [fields[axis].offset for axis in "xyz"],
                "itemsize": message.point_step,
            })
            points = np.ndarray(
                (message.height, message.width), dtype=dtype,
                buffer=message.data, strides=(message.row_step, message.point_step),
            )
            valid = np.ones(points.shape, dtype=bool)
            nonzero = np.zeros(points.shape, dtype=bool)
            for axis in "xyz":
                valid &= np.isfinite(points[axis])
                nonzero |= points[axis] != 0
            if not np.any(valid & nonzero):
                return
        except (TypeError, ValueError):
            return
        with self._lock:
            self._cloud_stamp = self._stamp(message)
            self._cloud_received = self._clock()

    def begin_refresh(self):
        """Establish an observation interval for one stationary planning request."""
        with self._lock:
            self._began_at = self._clock()
            self._cloud_after_ros = None
            self._anchor = self._odometry
            return self.motion_problem()

    def map_cleared(self):
        """Require a new acquisition after acknowledgment, keeping the base anchor."""
        with self._lock:
            self._cloud_after_ros = self._ros_now()

    def motion_problem(self):
        """Return why current base observations invalidate this planning interval."""
        with self._lock:
            message = self._odometry
            if (message is None or self._state_received is None
                    or self._clock() - self._state_received > self._max_state_age
                    or not 0 <= (self._ros_now() - self._stamp(message)) / 1e9 <= self._max_state_age):
                return "Fresh base odometry is required for environmental planning"
            if self._began_at is None or self._last_motion >= self._began_at:
                return "Base must remain stationary during environmental planning"
            if self._clock() - self._last_motion < self._settle_time:
                return "Waiting for the base to settle before environmental planning"
            anchor = self._anchor
            if (anchor is None or message.header.frame_id != anchor.header.frame_id
                    or message.child_frame_id != anchor.child_frame_id):
                return "Base reference frame changed during environmental planning"
            before, now = anchor.pose.pose, message.pose.pose
            xyz = [getattr(now.position, axis) - getattr(before.position, axis) for axis in "xyz"]
            q1 = np.array([getattr(before.orientation, axis) for axis in "xyzw"])
            q2 = np.array([getattr(now.orientation, axis) for axis in "xyzw"])
            if not np.isfinite(xyz).all() or not np.isfinite(q1).all() or not np.isfinite(q2).all():
                return "Base odometry contains invalid geometry"
            norm = np.linalg.norm(q1) * np.linalg.norm(q2)
            if norm < 1e-9:
                return "Base odometry contains an invalid orientation"
            angle = 2 * math.acos(min(1.0, abs(float(np.dot(q1, q2) / norm))))
            if math.hypot(*xyz) > self._translation_limit or angle > self._rotation_limit:
                return "Base moved since the local obstacle map was refreshed"
            return ""

    def has_fresh_cloud(self):
        with self._lock:
            return (
                self._cloud_after_ros is not None and self._cloud_stamp is not None
                and self._cloud_stamp >= self._cloud_after_ros
                and self._cloud_received >= self._began_at
                and 0 <= (self._ros_now() - self._cloud_stamp) / 1e9 <= self._max_cloud_age
                and self._clock() - self._cloud_received <= self._max_cloud_age
            )

    def destroy(self):
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        self._subscriptions.clear()
