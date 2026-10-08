"""Measured walking-height readiness, independent of saved mobility settings.

A successful zero-offset stand establishes the reference height. Comparing body
height relative to feet_center avoids mistaking a change in terrain elevation
for a body-height adjustment. No fixed robot/model height is assumed.
"""

from dataclasses import dataclass
import math

from synchros2.utilities import namespace_with


@dataclass(frozen=True)
class BodyHeightSample:
    height_m: float
    stamp_sec: float


class BodyHeightSource:
    def __init__(self, tf_listener, robot_name=""):
        self.tf_listener = tf_listener
        self.robot_name = robot_name

    def sample(self):
        transform = self.tf_listener.lookup_a_tform_b(
            namespace_with(self.robot_name, "feet_center"),
            namespace_with(self.robot_name, "body"),
            timeout_sec=0.0,
        )
        height = float(transform.transform.translation.z)
        stamp = (float(transform.header.stamp.sec)
                 + float(transform.header.stamp.nanosec) * 1e-9)
        return BodyHeightSample(height, stamp)


class BodyHeightReadiness:
    """Keep a measured nominal reference and verify post-reset settling."""

    def __init__(self, source, tolerance_m=0.01, settle_sec=0.3,
                 maximum_age_sec=0.5, settle_tolerance_m=0.005):
        self.source = source
        for value in (tolerance_m, settle_sec, maximum_age_sec, settle_tolerance_m):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Height readiness limits must be positive and finite")
        self.tolerance_m = tolerance_m
        self.settle_sec = settle_sec
        self.maximum_age_sec = maximum_age_sec
        self.settle_tolerance_m = settle_tolerance_m
        self.nominal_height_m = None
        self.reset_required = False
        self.last_error = "No height sample received"
        self.begin_confirmation(0.0)

    def sample(self, ros_time_sec):
        try:
            sample = self.source.sample()
            # TF can advance during lookup: compare against time read afterward.
            ros_now = ros_time_sec()
        except Exception as exception:
            self.last_error = f"{type(exception).__name__}: {exception}"
            return None
        if sample is None:
            self.last_error = "No height sample received"
            return None
        values = (sample.height_m, sample.stamp_sec, ros_now)
        if not all(math.isfinite(value) for value in values):
            self.last_error = "Non-finite height or timestamp"
            return None
        age = ros_now - sample.stamp_sec
        if not 0 <= age <= self.maximum_age_sec:
            self.last_error = (
                f"TF age={age:+.3f} s, allowed=0..{self.maximum_age_sec:.3f} s "
                f"(stamp={sample.stamp_sec:.6f}, now={ros_now:.6f})"
            )
            return None
        self.last_error = ""
        return sample

    def at_nominal_height(self, sample):
        return (not self.reset_required and self.nominal_height_m is not None
                and abs(sample.height_m - self.nominal_height_m) <= self.tolerance_m)

    def require_reset(self):
        # Conservatively retain this across rejected/cancelled height changes.
        self.reset_required = True

    def begin_confirmation(self, after_stamp):
        self._after_stamp = after_stamp
        self._anchor = None
        self._last_stamp = None
        self._stable_since = None

    def confirm_reset(self, ros_time_sec):
        sample = self.sample(ros_time_sec)
        if sample is None or sample.stamp_sec <= self._after_stamp:
            self._anchor = self._stable_since = None
            return False
        if self._last_stamp is not None and sample.stamp_sec <= self._last_stamp:
            return False
        if (self._last_stamp is not None
                and sample.stamp_sec - self._last_stamp > self.maximum_age_sec):
            self._anchor = self._stable_since = None
        self._last_stamp = sample.stamp_sec
        if (self.nominal_height_m is not None
                and abs(sample.height_m - self.nominal_height_m) > self.tolerance_m):
            self._anchor = self._stable_since = None
            return False
        if (self._anchor is None
                or abs(sample.height_m - self._anchor) > self.settle_tolerance_m):
            self._anchor = sample.height_m
            self._stable_since = sample.stamp_sec
            return False
        if sample.stamp_sec - self._stable_since < self.settle_sec:
            return False
        if self.nominal_height_m is None:
            self.nominal_height_m = sample.height_m
        self.reset_required = False
        return True
