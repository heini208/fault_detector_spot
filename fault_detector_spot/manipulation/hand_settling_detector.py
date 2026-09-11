"""Detect when Spot's hand is physically settled."""

from dataclasses import dataclass
from enum import Enum
import math
import time

from bosdyn.client.frame_helpers import HAND_FRAME_NAME, VISION_FRAME_NAME


LINEAR_VELOCITY_THRESHOLD_PARAMETER = (
    "arm.settling.linear_velocity_threshold_mps"
)
ANGULAR_VELOCITY_THRESHOLD_PARAMETER = (
    "arm.settling.angular_velocity_threshold_rad_s"
)
STABLE_DURATION_PARAMETER = "arm.settling.stable_duration_sec"
SETTLING_TIMEOUT_PARAMETER = "arm.settling.timeout_sec"

DEFAULT_LINEAR_VELOCITY_THRESHOLD_MPS = 0.01
DEFAULT_ANGULAR_VELOCITY_THRESHOLD_RAD_S = 0.05
DEFAULT_STABLE_DURATION_SEC = 0.40
DEFAULT_SETTLING_TIMEOUT_SEC = 3.0


class HandSettlingOutcome(str, Enum):
    RUNNING = "running"
    SETTLED = "settled"
    TIMEOUT = "timeout"
    SENSING_UNAVAILABLE = "sensing_unavailable"


@dataclass(frozen=True)
class HandSettlingUpdate:
    outcome: HandSettlingOutcome
    detail: str
    source: str = ""
    linear_speed_mps: float | None = None
    angular_speed_rad_s: float | None = None


class HandSettlingDetector:
    """Require continuously low measured hand motion before settling."""

    def __init__(
        self,
        arm_state_source,
        tf_listener,
        linear_velocity_threshold_mps: float = (
            DEFAULT_LINEAR_VELOCITY_THRESHOLD_MPS
        ),
        angular_velocity_threshold_rad_s: float = (
            DEFAULT_ANGULAR_VELOCITY_THRESHOLD_RAD_S
        ),
        stable_duration_sec: float = DEFAULT_STABLE_DURATION_SEC,
        timeout_sec: float = DEFAULT_SETTLING_TIMEOUT_SEC,
        monotonic_clock=time.monotonic,
    ):
        if arm_state_source is None:
            raise RuntimeError(
                "HandSettlingDetector requires an arm state source"
            )
        if tf_listener is None:
            raise RuntimeError(
                "HandSettlingDetector requires a TF listener"
            )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.arm_state_source = arm_state_source
        self.tf_listener = tf_listener
        self.linear_velocity_threshold_mps = self._positive_finite(
            linear_velocity_threshold_mps,
            "Settling linear velocity threshold",
        )
        self.angular_velocity_threshold_rad_s = self._positive_finite(
            angular_velocity_threshold_rad_s,
            "Settling angular velocity threshold",
        )
        self.stable_duration_sec = self._positive_finite(
            stable_duration_sec,
            "Settling stable duration",
        )
        self.timeout_sec = self._positive_finite(
            timeout_sec,
            "Settling timeout",
        )
        if self.timeout_sec < self.stable_duration_sec:
            raise ValueError(
                "Settling timeout must be at least the stable duration"
            )

        self._monotonic_clock = monotonic_clock
        self.reset()

    @classmethod
    def from_node(
        cls,
        node,
        arm_state_source,
        tf_listener,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError(
                "HandSettlingDetector requires a ROS node"
            )

        values = {}
        for name, default in (
            (
                LINEAR_VELOCITY_THRESHOLD_PARAMETER,
                DEFAULT_LINEAR_VELOCITY_THRESHOLD_MPS,
            ),
            (
                ANGULAR_VELOCITY_THRESHOLD_PARAMETER,
                DEFAULT_ANGULAR_VELOCITY_THRESHOLD_RAD_S,
            ),
            (
                STABLE_DURATION_PARAMETER,
                DEFAULT_STABLE_DURATION_SEC,
            ),
            (
                SETTLING_TIMEOUT_PARAMETER,
                DEFAULT_SETTLING_TIMEOUT_SEC,
            ),
        ):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            values[name] = float(node.get_parameter(name).value)

        return cls(
            arm_state_source=arm_state_source,
            tf_listener=tf_listener,
            linear_velocity_threshold_mps=values[
                LINEAR_VELOCITY_THRESHOLD_PARAMETER
            ],
            angular_velocity_threshold_rad_s=values[
                ANGULAR_VELOCITY_THRESHOLD_PARAMETER
            ],
            stable_duration_sec=values[STABLE_DURATION_PARAMETER],
            timeout_sec=values[SETTLING_TIMEOUT_PARAMETER],
            monotonic_clock=monotonic_clock,
        )

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> HandSettlingUpdate:
        """Start a fresh bounded settling observation."""
        self.reset()
        self._active = True
        self._started_at = self._monotonic_clock()
        return self.poll()

    def poll(self) -> HandSettlingUpdate:
        """Advance settling detection without blocking."""
        if not self._active:
            raise RuntimeError("No settling observation is active")

        now = self._monotonic_clock()
        velocity_sample = self.arm_state_source.hand_velocity_sample()

        if velocity_sample is not None:
            self._pose_sample = None
            if (
                velocity_sample.received_at
                == self._last_velocity_received_at
            ):
                return self._running_or_timeout(
                    now,
                    "Waiting for a new measured hand velocity sample",
                    source="velocity",
                )

            self._last_velocity_received_at = (
                velocity_sample.received_at
            )
            return self._record_motion(
                now,
                source="velocity",
                linear_speed_mps=velocity_sample.linear_speed_mps,
                angular_speed_rad_s=(
                    velocity_sample.angular_speed_rad_s
                ),
            )

        pose_measurement = self._pose_motion_measurement(now)
        if pose_measurement is None:
            return self._running_or_timeout(
                now,
                "Waiting for measured hand motion",
                source="pose",
            )

        return self._record_motion(
            now,
            source="pose",
            linear_speed_mps=pose_measurement[0],
            angular_speed_rad_s=pose_measurement[1],
        )

    def reset(self) -> None:
        self._active = False
        self._started_at = None
        self._stable_since = None
        self._stable_source = ""
        self._last_velocity_received_at = None
        self._pose_sample = None
        self._had_motion_measurement = False

    def _pose_motion_measurement(self, now: float):
        try:
            transform = self.tf_listener.lookup_a_tform_b(
                VISION_FRAME_NAME,
                HAND_FRAME_NAME,
                timeout_sec=0.0,
            )
        except Exception:
            self._pose_sample = None
            if self._stable_source == "pose":
                self._stable_since = None
                self._stable_source = ""
            return None

        pose = self._transform_values(transform)
        previous = self._pose_sample
        self._pose_sample = (now, pose)
        if previous is None:
            if self._stable_source != "pose":
                self._stable_since = None
                self._stable_source = ""
            return None

        previous_time, previous_pose = previous
        elapsed = now - previous_time
        if elapsed <= 1e-6:
            return None

        linear_distance = math.sqrt(
            sum(
                (current - old) * (current - old)
                for current, old in zip(
                    pose[:3],
                    previous_pose[:3],
                )
            )
        )
        angular_distance = self._rotation_distance_rad(
            previous_pose[3:],
            pose[3:],
        )
        return (
            linear_distance / elapsed,
            angular_distance / elapsed,
        )

    def _record_motion(
        self,
        now: float,
        source: str,
        linear_speed_mps: float,
        angular_speed_rad_s: float,
    ) -> HandSettlingUpdate:
        linear_speed = float(linear_speed_mps)
        angular_speed = float(angular_speed_rad_s)
        if not (
            math.isfinite(linear_speed)
            and math.isfinite(angular_speed)
        ):
            self._break_stability(source)
            return self._running_or_timeout(
                now,
                "Measured hand motion is not finite",
                source=source,
            )

        self._had_motion_measurement = True
        if self._stable_source != source:
            self._stable_since = None
            self._stable_source = source

        is_stable = (
            linear_speed
            <= self.linear_velocity_threshold_mps
            and angular_speed
            <= self.angular_velocity_threshold_rad_s
        )
        if not is_stable:
            self._stable_since = None
            return self._running_or_timeout(
                now,
                "Hand is still moving",
                source=source,
                linear_speed_mps=linear_speed,
                angular_speed_rad_s=angular_speed,
            )

        if self._stable_since is None:
            self._stable_since = now

        stable_for = now - self._stable_since
        if stable_for >= self.stable_duration_sec:
            self._active = False
            return HandSettlingUpdate(
                HandSettlingOutcome.SETTLED,
                "Hand remained below settling thresholds for "
                f"{self.stable_duration_sec:.2f} s",
                source=source,
                linear_speed_mps=linear_speed,
                angular_speed_rad_s=angular_speed,
            )

        return self._running_or_timeout(
            now,
            "Hand is below settling thresholds for "
            f"{stable_for:.2f} s",
            source=source,
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
        )

    def _running_or_timeout(
        self,
        now: float,
        detail: str,
        source: str = "",
        linear_speed_mps: float = None,
        angular_speed_rad_s: float = None,
    ) -> HandSettlingUpdate:
        if (
            self._started_at is not None
            and now - self._started_at >= self.timeout_sec
        ):
            self._active = False
            if self._had_motion_measurement:
                outcome = HandSettlingOutcome.TIMEOUT
                timeout_detail = (
                    "Hand did not remain settled within "
                    f"{self.timeout_sec:.1f} s"
                )
            else:
                outcome = HandSettlingOutcome.SENSING_UNAVAILABLE
                timeout_detail = (
                    "Measured hand motion was unavailable for "
                    f"{self.timeout_sec:.1f} s"
                )
            return HandSettlingUpdate(
                outcome,
                timeout_detail,
                source=source,
                linear_speed_mps=linear_speed_mps,
                angular_speed_rad_s=angular_speed_rad_s,
            )

        return HandSettlingUpdate(
            HandSettlingOutcome.RUNNING,
            detail,
            source=source,
            linear_speed_mps=linear_speed_mps,
            angular_speed_rad_s=angular_speed_rad_s,
        )

    def _break_stability(self, source: str) -> None:
        if self._stable_source == source:
            self._stable_since = None
            self._stable_source = ""

    @staticmethod
    def _transform_values(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Hand transform contains non-finite values")
        return values

    @staticmethod
    def _rotation_distance_rad(first, second) -> float:
        first_normalized = HandSettlingDetector._normalized_quaternion(
            first
        )
        second_normalized = HandSettlingDetector._normalized_quaternion(
            second
        )
        dot = abs(
            sum(
                a * b
                for a, b in zip(
                    first_normalized,
                    second_normalized,
                )
            )
        )
        dot = min(1.0, max(0.0, dot))
        return 2.0 * math.acos(dot)

    @staticmethod
    def _normalized_quaternion(values):
        values = tuple(float(value) for value in values)
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError("Hand transform quaternion is invalid")
        return tuple(value / norm for value in values)

    @staticmethod
    def _positive_finite(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized


__all__ = [
    "ANGULAR_VELOCITY_THRESHOLD_PARAMETER",
    "DEFAULT_ANGULAR_VELOCITY_THRESHOLD_RAD_S",
    "DEFAULT_LINEAR_VELOCITY_THRESHOLD_MPS",
    "DEFAULT_SETTLING_TIMEOUT_SEC",
    "DEFAULT_STABLE_DURATION_SEC",
    "HandSettlingDetector",
    "HandSettlingOutcome",
    "HandSettlingUpdate",
    "LINEAR_VELOCITY_THRESHOLD_PARAMETER",
    "SETTLING_TIMEOUT_PARAMETER",
    "STABLE_DURATION_PARAMETER",
]
