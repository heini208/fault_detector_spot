"""Continuous speed-aware force threshold for guarded arm motion."""

import math

from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)


class SpeedAwareForceContactPolicy:
    """Scale force tolerance continuously with planned arm speed."""

    def __init__(
        self,
        minimum_threshold_n=None,
        reference_speed_mps=None,
        reference_threshold_n=None,
        maximum_threshold_n=None,
        consecutive_samples=None,
        config=None,
    ):
        config = config if config is not None else ArmMotionParameters()
        minimum_threshold_n = config.get(
            "contact.minimum_force_delta_threshold_n", minimum_threshold_n
        )
        reference_speed_mps = config.get(
            "contact.reference_linear_speed_mps", reference_speed_mps
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
        self.reference_speed_mps = self._positive(
            reference_speed_mps,
            "Reference arm speed",
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

    def threshold_for(self, linear_speed_mps: float) -> float:
        """Return the bounded threshold for one planned translational speed."""
        speed = float(linear_speed_mps)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(
                "Guarded arm speed must be positive and finite"
            )

        speed_allowance_n = (
            self.reference_threshold_n
            - self.minimum_threshold_n
        ) * (speed / self.reference_speed_mps)
        threshold = self.minimum_threshold_n + speed_allowance_n
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


__all__ = [
    "SpeedAwareForceContactPolicy",
]
