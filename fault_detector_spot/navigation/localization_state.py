"""Map-alignment evidence and fail-closed mapping tracking state.

Process liveness and pose covariance alone are not proof of map alignment.
Only fresh, matched, accepted lidar registrations establish LOCALIZED.
"""
from enum import Enum
from threading import RLock
import time

from fault_detector_spot.navigation.localization_quality import (
    LocalizationQualityConfig,
    accepted_lidar_localization,
    assess_localization_quality,
)


class LocalizationState(str, Enum):
    UNLOCALIZED = "unlocalized"
    LOCALIZING = "localizing"
    LOCALIZED = "localized"
    LOST = "lost"


def _stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class LocalizationStateTracker:
    """Track one RTAB-Map process and map, never carrying evidence across sessions."""

    def __init__(
        self, ros_clock_ns, *, steady_clock=time.monotonic,
        required_matches=3, maximum_age_sec=1.5, stream_timeout_sec=3.0,
    ):
        self._ros_clock_ns = ros_clock_ns
        self._steady_clock = steady_clock
        self.required_matches = required_matches
        self.maximum_age_sec = maximum_age_sec
        self.stream_timeout_sec = stream_timeout_sec
        self._quality = LocalizationQualityConfig(maximum_age_sec=maximum_age_sec)
        self._lock = RLock()
        self.reset()

    def reset(self):
        with self._lock:
            self.map_name = None
            self.process = None
            self.mode = "none"
            self.state = LocalizationState.UNLOCALIZED
            self.reason = "No active localization session"
            self._pose = None
            self._info = None
            self._pose_stamp = 0
            self._info_stamp = 0
            self._pose_at = 0.0
            self._info_at = 0.0
            self._last_checked_stamp = 0
            self._minimum_stamp = 0
            self._accepted_matches = 0
            self._last_accepted_at = 0.0
            self._mapping_since = 0.0
            self._mapping_pose_boundary = 0.0
            self._mapping_info_boundary = 0.0
            self._paused = False

    def begin_localization(self, map_name, process):
        with self._lock:
            self.reset()
            self.map_name = map_name
            self.process = process
            self.mode = "localization"
            self.state = LocalizationState.LOCALIZING
            self.reason = "Awaiting fresh, accepted lidar-to-map registrations"
            self._minimum_stamp = int(self._ros_clock_ns())

    def begin_mapping(self, map_name, process, *, existing_map):
        with self._lock:
            if existing_map:
                allowed, reason = self.can_extend(map_name, process)
                if not allowed:
                    raise RuntimeError(f"Cannot extend map: {reason}")
            else:
                self.reset()
                self.map_name = map_name
                self.process = process
                self.state = LocalizationState.UNLOCALIZED
                self.reason = "Creating a new map, no prior global alignment"
            self.mode = "mapping"
            self._mapping_since = self._steady_clock()
            self._mapping_pose_boundary = self._pose_at
            self._mapping_info_boundary = self._info_at
            self._paused = False

    def observe_pose(self, message):
        with self._lock:
            if self.mode == "none":
                return
            try:
                stamp = _stamp_ns(message)
            except (AttributeError, TypeError, ValueError):
                return
            if stamp <= self._minimum_stamp or stamp < self._pose_stamp:
                return
            if stamp > self._pose_stamp:
                self._pose_stamp = stamp
                self._pose_at = self._steady_clock()
            self._pose = message
            self._check_registration()

    def observe_info(self, message):
        with self._lock:
            if self.mode == "none":
                return
            try:
                stamp = _stamp_ns(message)
            except (AttributeError, TypeError, ValueError):
                return
            if stamp <= self._minimum_stamp or stamp < self._info_stamp:
                return
            if stamp > self._info_stamp:
                self._info_stamp = stamp
                self._info_at = self._steady_clock()
            self._info = message
            self._check_registration()

    def _pose_error(self):
        if self._pose is None:
            return "No localization pose"
        try:
            return assess_localization_quality(
                self._pose, int(self._ros_clock_ns()), self._quality,
            ).detail
        except (TypeError, ValueError, OverflowError):
            return "Localization clock is invalid"

    def _check_registration(self):
        if self.mode != "localization" or self._pose is None or self._info is None:
            return
        if self._pose_stamp != self._info_stamp or self._pose_stamp <= self._last_checked_stamp:
            return
        self._last_checked_stamp = self._pose_stamp
        error = self._pose_error()
        if (not error and self._info.header.frame_id == "map"
                and accepted_lidar_localization(self._info)):
            now = self._steady_clock()
            if self._last_accepted_at and now - self._last_accepted_at > self.stream_timeout_sec:
                self._accepted_matches = 0
            self._accepted_matches += 1
            self._last_accepted_at = now
            if self._accepted_matches >= self.required_matches:
                self.state = LocalizationState.LOCALIZED
                self.reason = "Fresh accepted lidar-to-map registrations confirmed"
            return
        self._accepted_matches = 0
        if self.state != LocalizationState.LOCALIZED:
            self.reason = error or "Lidar-to-map registration not accepted"

    def observe_seed(self, _message):
        with self._lock:
            if self.mode == "none":
                return
            self._minimum_stamp = max(
                self._minimum_stamp, int(self._ros_clock_ns()),
                self._pose_stamp, self._info_stamp,
            )
            self._accepted_matches = 0
            self._last_accepted_at = 0.0
            if self.mode == "mapping":
                self.state = LocalizationState.LOST
                self.reason = "Pose reset received while mapping"
            else:
                self.state = LocalizationState.LOCALIZING
                self.reason = "Pose reset requires new accepted map registrations"

    def _refresh(self):
        now = self._steady_clock()
        if self.mode == "localization" and self.state == LocalizationState.LOCALIZED:
            if now - self._last_accepted_at > self.stream_timeout_sec:
                self.state = LocalizationState.LOST
                self.reason = "Accepted lidar-to-map registration stream is stale"
        if self.mode == "mapping" and not self._paused:
            if now - self._mapping_since < self.stream_timeout_sec:
                return
            if (self._pose_at <= self._mapping_pose_boundary
                    or self._info_at <= self._mapping_info_boundary
                    or now - self._pose_at > self.stream_timeout_sec
                    or now - self._info_at > self.stream_timeout_sec):
                self.state = LocalizationState.LOST
                self.reason = "RTAB-Map pose or processed-scan stream stopped advancing"
                return
            error = self._pose_error()
            if error:
                self.state = LocalizationState.LOST
                self.reason = f"Mapping pose became unreliable: {error}"

    def snapshot(self):
        with self._lock:
            self._refresh()
            return self.state, self.reason, self._paused

    def can_extend(self, map_name, process):
        with self._lock:
            self._refresh()
            if (self.mode != "localization" or self.process is not process
                    or self.map_name != map_name):
                return False, "matching active localization session required"
            if self.state != LocalizationState.LOCALIZED:
                return False, self.reason
            return True, ""

    def requires_pause(self, process):
        with self._lock:
            self._refresh()
            return (self.mode == "mapping" and self.process is process
                    and self.state == LocalizationState.LOST and not self._paused)

    def pause_confirmed(self, process):
        with self._lock:
            if self.process is process and self.mode == "mapping":
                self._paused = True
