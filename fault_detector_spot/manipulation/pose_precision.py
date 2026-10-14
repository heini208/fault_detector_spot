"""Pure pose stability and bounded correction rules shared by arm executions."""

from copy import deepcopy
import math

from fault_detector_spot.shared.geometry.rotation import rotation_distance_rad


class PoseStabilityWindow:
    """Require advancing acquisition stamps and a stationary recent pose window."""

    def __init__(self, *, minimum_samples=3, minimum_span_sec=0.3,
                 position_tolerance_m=0.001,
                 orientation_tolerance_rad=math.radians(0.2),
                 maximum_sample_gap_sec=0.25, pose_names=None):
        if type(minimum_samples) is not int or minimum_samples < 3:
            raise ValueError("Pose stability requires at least three samples")
        for value in (minimum_span_sec, position_tolerance_m,
                      orientation_tolerance_rad, maximum_sample_gap_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Pose stability limits must be positive and finite")
        self.minimum_samples = minimum_samples
        self.minimum_span_sec = minimum_span_sec
        self.position_tolerance_m = position_tolerance_m
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.maximum_sample_gap_sec = maximum_sample_gap_sec
        self.pose_names = None if pose_names is None else tuple(str(name).strip() for name in pose_names)
        if self.pose_names is not None and (not self.pose_names or not all(self.pose_names)):
            raise ValueError("Pose stability names must not be empty")
        self.reset()

    def reset(self):
        self._last_stamp = 0.0
        self._samples = []
        self._last_reset_reason = ""

    def invalidate(self, reason="Pose feedback was unavailable or invalid"):
        self._samples.clear()
        self._last_reset_reason = str(reason)

    @property
    def last_stamp_seconds(self):
        return self._last_stamp

    @property
    def last_reset_reason(self):
        return self._last_reset_reason

    @property
    def sample_count(self):
        return len(self._samples)

    @property
    def span_sec(self):
        return self._samples[-1][0] - self._samples[0][0] if self._samples else 0.0

    def add(self, stamp_seconds, poses):
        try:
            if not math.isfinite(stamp_seconds) or stamp_seconds <= 0:
                raise ValueError("Pose acquisition timestamp is invalid")
            poses = tuple(poses)
            if not poses:
                raise ValueError("Pose stability requires a pose")
            if self.pose_names is not None and len(poses) != len(self.pose_names):
                raise ValueError("Pose stability names do not match sample dimensions")
            for pose in poses:
                pose.validate()
        except Exception as exception:
            self.invalidate(str(exception))
            raise
        if stamp_seconds <= self._last_stamp:
            raise ValueError("Waiting for a distinct, advancing pose acquisition timestamp")
        self._last_stamp = stamp_seconds
        if self._samples and stamp_seconds - self._samples[-1][0] > self.maximum_sample_gap_sec:
            self.invalidate(
                f"Pose acquisition gap {stamp_seconds - self._samples[-1][0]:.3f} s "
                f"exceeded {self.maximum_sample_gap_sec:.3f} s limit")
        for _, previous in self._samples:
            if len(previous) != len(poses):
                self.invalidate("Pose stability sample dimensions changed")
                raise ValueError("Pose stability sample dimensions changed")
            changed = False
            for index, (a, b) in enumerate(zip(poses, previous)):
                position_change = math.dist(
                    (a.position.x, a.position.y, a.position.z),
                    (b.position.x, b.position.y, b.position.z))
                orientation_change = rotation_distance_rad(a.orientation, b.orientation)
                if (position_change > self.position_tolerance_m
                        or orientation_change > self.orientation_tolerance_rad):
                    name = self.pose_names[index] if self.pose_names is not None else f"pose {index + 1}"
                    self.invalidate(
                        f"{name} changed {position_change * 1000:.2f} mm / "
                        f"{math.degrees(orientation_change):.3f} deg "
                        f"(limits {self.position_tolerance_m * 1000:.2f} mm / "
                        f"{math.degrees(self.orientation_tolerance_rad):.3f} deg)")
                    changed = True
                    break
            if changed:
                break
        self._samples.append((stamp_seconds, deepcopy(poses)))
        while (len(self._samples) > self.minimum_samples
               and stamp_seconds - self._samples[1][0] >= self.minimum_span_sec):
            self._samples.pop(0)
        return (len(self._samples) >= self.minimum_samples
                and self.span_sec + 1e-9 >= self.minimum_span_sec)


class PoseCorrectionBudget:
    """Reserve each correction only while error improves within travel limits."""

    def __init__(self, *, maximum_corrections=3, maximum_translation_m=0.020,
                 maximum_rotation_rad=math.radians(5.0),
                 maximum_total_translation_m=0.040,
                 maximum_total_rotation_rad=math.radians(10.0),
                 minimum_improvement_fraction=0.10):
        if type(maximum_corrections) is not int or maximum_corrections < 0:
            raise ValueError("Maximum pose corrections must be a non-negative integer")
        for value in (maximum_translation_m, maximum_rotation_rad,
                      maximum_total_translation_m, maximum_total_rotation_rad,
                      minimum_improvement_fraction):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Pose correction limits must be positive and finite")
        if minimum_improvement_fraction >= 1:
            raise ValueError("Pose correction improvement fraction must be less than one")
        self.maximum_corrections = maximum_corrections
        self.maximum_translation_m = maximum_translation_m
        self.maximum_rotation_rad = maximum_rotation_rad
        self.maximum_total_translation_m = maximum_total_translation_m
        self.maximum_total_rotation_rad = maximum_total_rotation_rad
        self.minimum_improvement_fraction = minimum_improvement_fraction
        self.corrections = 0
        self.total_translation_m = 0.0
        self.total_rotation_rad = 0.0
        self.previous_error_score = None

    def reserve(self, position_error_m, orientation_error_rad,
                position_tolerance_m, orientation_tolerance_rad):
        if any(not math.isfinite(value) or value < 0
               for value in (position_error_m, orientation_error_rad)):
            raise ValueError("Pose errors must be finite and nonnegative")
        if any(not math.isfinite(value) or value <= 0
               for value in (position_tolerance_m, orientation_tolerance_rad)):
            raise ValueError("Pose tolerances must be positive and finite")
        if self.corrections >= self.maximum_corrections:
            raise ValueError("Pose correction limit reached")
        if (position_error_m > self.maximum_translation_m
                or orientation_error_rad > self.maximum_rotation_rad):
            raise ValueError("Pose error is outside the bounded correction range")
        score = max(position_error_m / position_tolerance_m,
                    orientation_error_rad / orientation_tolerance_rad)
        if (self.previous_error_score is not None
                and score > self.previous_error_score * (1 - self.minimum_improvement_fraction)):
            raise ValueError(
                f"Pose error did not improve enough after correction {self.corrections} "
                f"(normalized error {self.previous_error_score:.3f} → {score:.3f}; "
                f"requires {self.minimum_improvement_fraction:.0%} improvement)")
        translation = self.total_translation_m + position_error_m
        rotation = self.total_rotation_rad + orientation_error_rad
        if (translation > self.maximum_total_translation_m
                or rotation > self.maximum_total_rotation_rad):
            raise ValueError("Total pose correction travel budget exceeded")
        self.corrections += 1
        self.total_translation_m = translation
        self.total_rotation_rad = rotation
        self.previous_error_score = score
