"""Qualify distinct observations of a fixed full 3D landmark transform."""

from collections import deque
from copy import deepcopy
import math

from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.shared.geometry.rotation import rotation_distance_rad


class LandmarkPoseWindow:
    """Return the latest pose after a bounded, stable observation window."""

    def __init__(
        self, required_samples, maximum_sample_span_sec,
        maximum_position_spread_m, maximum_rotation_spread_rad,
    ):
        if type(required_samples) is not int or required_samples < 2:
            raise ValueError("Landmark required_samples must be an integer >= 2")
        for value in (
            maximum_sample_span_sec, maximum_position_spread_m,
            maximum_rotation_spread_rad,
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Landmark stability limits must be positive and finite")
        self.required_samples = required_samples
        self.maximum_sample_span_ns = int(maximum_sample_span_sec * 1e9)
        self.maximum_position_spread_m = maximum_position_spread_m
        self.maximum_rotation_spread_rad = maximum_rotation_spread_rad
        self._samples = deque(maxlen=required_samples)
        self._source_frame = None
        self.detail = "Waiting for distinct stable landmark observations"

    @property
    def sample_count(self):
        return len(self._samples)

    def clear(self):
        self._samples.clear()
        self._source_frame = None
        self.detail = "Waiting for distinct stable landmark observations"

    def add(self, stamp_ns: int, pose: PoseData, source_frame: str = ""):
        try:
            if type(stamp_ns) is not int or stamp_ns <= 0:
                raise ValueError("Landmark acquisition timestamp must be positive")
            if not isinstance(pose, PoseData):
                raise TypeError("Landmark stability requires PoseData")
            pose.validate()
        except (TypeError, ValueError):
            self.clear()
            raise
        if self._samples and (
            self._source_frame != source_frame
            or stamp_ns < self._samples[-1][0]
        ):
            self.clear()
        if self._samples and stamp_ns == self._samples[-1][0]:
            return self._qualified_pose()
        self._source_frame = source_frame
        self._samples.append((stamp_ns, deepcopy(pose)))
        while stamp_ns - self._samples[0][0] > self.maximum_sample_span_ns:
            self._samples.popleft()
        for index, (_, first) in enumerate(self._samples):
            for _, second in tuple(self._samples)[index + 1:]:
                position_error = math.dist(
                    (first.position.x, first.position.y, first.position.z),
                    (second.position.x, second.position.y, second.position.z),
                )
                rotation_error = rotation_distance_rad(first.orientation, second.orientation)
                if (position_error > self.maximum_position_spread_m
                        or rotation_error > self.maximum_rotation_spread_rad):
                    latest = self._samples[-1]
                    self._samples.clear()
                    self._samples.append(latest)
                    self.detail = (
                        "Landmark pose is unstable "
                        f"({position_error:.3f} m, {math.degrees(rotation_error):.1f} deg); "
                        "wait for consistent observations"
                    )
                    return None
        return self._qualified_pose()

    def _qualified_pose(self):
        if self.sample_count < self.required_samples:
            self.detail = (
                f"Waiting for distinct stable landmark observations "
                f"({self.sample_count}/{self.required_samples})"
            )
            return None
        self.detail = ""
        return deepcopy(self._samples[-1][1])


__all__ = ["LandmarkPoseWindow"]
