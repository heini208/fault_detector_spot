"""Cartesian speed limits for Spot arm movements."""

from dataclasses import dataclass, field
import math

from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)


@dataclass(frozen=True)
class ArmMotionSpeed:
    """Linear and angular speed limits for one arm movement."""

    linear_speed_mps: float = field(
        default_factory=lambda: ArmMotionParameters().get(
            "motion.linear_speed_mps"
        )
    )
    angular_speed_rad_s: float = field(
        default_factory=lambda: ArmMotionParameters().get(
            "motion.angular_speed_rad_s"
        )
    )

    def __post_init__(self):
        _require_positive_finite(
            self.linear_speed_mps,
            "Linear arm speed",
        )
        _require_positive_finite(
            self.angular_speed_rad_s,
            "Angular arm speed",
        )


@dataclass(frozen=True)
class ArmMotionSpeedPolicy:
    """Convert controlled-frame travel and speed into Spot duration."""

    default_speed: ArmMotionSpeed = field(
        default_factory=ArmMotionSpeed
    )
    minimum_duration_sec: float = field(
        default_factory=lambda: ArmMotionParameters().get(
            "motion.minimum_duration_sec"
        )
    )

    def __post_init__(self):
        if not isinstance(self.default_speed, ArmMotionSpeed):
            raise TypeError(
                "Default arm speed must be an ArmMotionSpeed"
            )
        _require_positive_finite(
            self.minimum_duration_sec,
            "Minimum arm movement duration",
        )

    @classmethod
    def from_node(cls, node):
        """Read the executor default speed from ROS parameters."""
        if node is None:
            raise RuntimeError(
                "ArmMotionSpeedPolicy requires a ROS node"
            )

        return cls.from_config(ArmMotionParameters(node))

    @classmethod
    def from_config(cls, config):
        """Use one configuration for all speed and duration settings."""
        return cls(
            default_speed=ArmMotionSpeed(
                linear_speed_mps=config.get("motion.linear_speed_mps"),
                angular_speed_rad_s=config.get("motion.angular_speed_rad_s"),
            ),
            minimum_duration_sec=config.get("motion.minimum_duration_sec"),
        )

    def duration_between(
        self,
        current_pose,
        target_pose,
        speed=None,
    ) -> float:
        """Return the duration required by the selected speed limits."""
        effective_speed = self._effective_speed(speed)
        translation_distance = self._translation_distance(
            current_pose,
            target_pose,
        )
        rotation_angle = self._rotation_angle(
            current_pose,
            target_pose,
        )
        return max(
            translation_distance / effective_speed.linear_speed_mps,
            rotation_angle / effective_speed.angular_speed_rad_s,
            self.minimum_duration_sec,
        )

    def _effective_speed(self, speed):
        if speed is None:
            return self.default_speed
        if not isinstance(speed, ArmMotionSpeed):
            raise TypeError(
                "Arm movement speed override must be an ArmMotionSpeed"
            )
        return speed

    @staticmethod
    def _translation_distance(current_pose, target_pose) -> float:
        dx = (
            float(target_pose.position.x)
            - float(current_pose.position.x)
        )
        dy = (
            float(target_pose.position.y)
            - float(current_pose.position.y)
        )
        dz = (
            float(target_pose.position.z)
            - float(current_pose.position.z)
        )
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if not math.isfinite(distance):
            raise ValueError(
                "Arm movement translation distance is not finite"
            )
        return distance

    @staticmethod
    def _rotation_angle(current_pose, target_pose) -> float:
        current = ArmMotionSpeedPolicy._normalized_quaternion(
            current_pose.orientation
        )
        target = ArmMotionSpeedPolicy._normalized_quaternion(
            target_pose.orientation
        )
        dot = abs(sum(a * b for a, b in zip(current, target)))
        dot = min(1.0, max(0.0, dot))
        return 2.0 * math.acos(dot)

    @staticmethod
    def _normalized_quaternion(quaternion):
        values = tuple(
            float(value)
            for value in (
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
            )
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Arm movement quaternion contains a non-finite value"
            )
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError(
                "Arm movement quaternion norm is zero"
            )
        return tuple(value / norm for value in values)


def _require_positive_finite(value, label):
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(
            f"{label} must be positive and finite"
        )


__all__ = [
    "ArmMotionSpeed",
    "ArmMotionSpeedPolicy",
]
