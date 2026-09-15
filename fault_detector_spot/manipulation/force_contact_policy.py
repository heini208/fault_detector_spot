"""Continuous motion-aware force threshold for guarded arm motion."""

import math

from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)


class SpeedAwareForceContactPolicy:
    """Scale force tolerance continuously with planned Cartesian motion."""

    def __init__(
        self,
        minimum_threshold_n=None,
        reference_linear_speed_mps=None,
        reference_angular_speed_rad_s=None,
        reference_threshold_n=None,
        maximum_threshold_n=None,
        consecutive_samples=None,
        config=None,
    ):
        config = config if config is not None else ArmMotionParameters()
        minimum_threshold_n = config.get(
            "contact.minimum_force_delta_threshold_n", minimum_threshold_n
        )
        reference_linear_speed_mps = config.get(
            "contact.reference_linear_speed_mps",
            reference_linear_speed_mps,
        )
        reference_angular_speed_rad_s = config.get(
            "contact.reference_angular_speed_rad_s",
            reference_angular_speed_rad_s,
        )
        reference_threshold_n = config.get(
            "contact.reference_force_delta_threshold_n", reference_threshold_n
        )
        maximum_threshold_n = config.get(
            "contact.maximum_force_delta_threshold_n", maximum_threshold_n
        )
        consecutive_samples = config.get(
            "contact.consecutive_samples", consecutive_samples
        )
        self.minimum_threshold_n = self._positive(
            minimum_threshold_n,
            "Minimum force threshold",
        )
        self.reference_linear_speed_mps = self._positive(
            reference_linear_speed_mps,
            "Reference linear arm speed",
        )
        self.reference_angular_speed_rad_s = self._positive(
            reference_angular_speed_rad_s,
            "Reference angular arm speed",
        )
        self.reference_threshold_n = self._positive(
            reference_threshold_n,
            "Reference force threshold",
        )
        self.maximum_threshold_n = self._positive(
            maximum_threshold_n,
            "Maximum force threshold",
        )
        if self.reference_threshold_n < self.minimum_threshold_n:
            raise ValueError(
                "Reference force threshold must not be below "
                "the minimum threshold"
            )
        if self.maximum_threshold_n < self.reference_threshold_n:
            raise ValueError(
                "Maximum force threshold must not be below "
                "the reference threshold"
            )
        if (
            isinstance(consecutive_samples, bool)
            or not isinstance(consecutive_samples, int)
            or consecutive_samples < 1
        ):
            raise ValueError(
                "Force contact consecutive sample count must be positive"
            )
        self.consecutive_samples = consecutive_samples

    @classmethod
    def from_node(cls, node):
        if node is None:
            raise RuntimeError("SpeedAwareForceContactPolicy requires a ROS node")
        return cls(
            config=ArmMotionParameters(node),
        )

    def threshold_for(
        self,
        linear_speed_mps: float,
        angular_speed_rad_s: float = 0.0,
    ) -> float:
        """Return the bounded threshold for planned linear and angular motion."""
        linear_speed = self._non_negative(
            linear_speed_mps,
            "Guarded arm linear speed",
        )
        angular_speed = self._non_negative(
            angular_speed_rad_s,
            "Guarded arm angular speed",
        )
        motion_scale = max(
            linear_speed / self.reference_linear_speed_mps,
            angular_speed / self.reference_angular_speed_rad_s,
        )
        motion_allowance_n = (
            self.reference_threshold_n
            - self.minimum_threshold_n
        ) * motion_scale
        threshold = self.minimum_threshold_n + motion_allowance_n
        return min(
            self.maximum_threshold_n,
            max(self.minimum_threshold_n, threshold),
        )

    @staticmethod
    def _positive(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized

    @staticmethod
    def _non_negative(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(
                f"{label} must be non-negative and finite"
            )
        return normalized


__all__ = [
    "SpeedAwareForceContactPolicy",
]
