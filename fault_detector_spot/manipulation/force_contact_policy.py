"""Continuous speed-aware force threshold for guarded arm motion."""

import math


CONTACT_MINIMUM_THRESHOLD_PARAMETER = (
    "arm.contact.minimum_force_delta_threshold_n"
)
CONTACT_REFERENCE_SPEED_PARAMETER = (
    "arm.contact.reference_linear_speed_mps"
)
CONTACT_REFERENCE_THRESHOLD_PARAMETER = (
    "arm.contact.reference_force_delta_threshold_n"
)
CONTACT_MAXIMUM_THRESHOLD_PARAMETER = (
    "arm.contact.maximum_force_delta_threshold_n"
)
CONTACT_CONSECUTIVE_SAMPLES_PARAMETER = (
    "arm.contact.consecutive_samples"
)

DEFAULT_CONTACT_MINIMUM_THRESHOLD_N = 3.0
DEFAULT_CONTACT_REFERENCE_SPEED_MPS = 0.005
DEFAULT_CONTACT_REFERENCE_THRESHOLD_N = 5.0
DEFAULT_CONTACT_MAXIMUM_THRESHOLD_N = 10.0
DEFAULT_CONTACT_CONSECUTIVE_SAMPLES = 2


class SpeedAwareForceContactPolicy:
    """Scale force tolerance continuously with planned arm speed."""

    def __init__(
        self,
        minimum_threshold_n: float = DEFAULT_CONTACT_MINIMUM_THRESHOLD_N,
        reference_speed_mps: float = DEFAULT_CONTACT_REFERENCE_SPEED_MPS,
        reference_threshold_n: float = (
            DEFAULT_CONTACT_REFERENCE_THRESHOLD_N
        ),
        maximum_threshold_n: float = DEFAULT_CONTACT_MAXIMUM_THRESHOLD_N,
        consecutive_samples: int = DEFAULT_CONTACT_CONSECUTIVE_SAMPLES,
    ):
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
            raise RuntimeError(
                "SpeedAwareForceContactPolicy requires a ROS node"
            )

        parameters = (
            (
                CONTACT_MINIMUM_THRESHOLD_PARAMETER,
                DEFAULT_CONTACT_MINIMUM_THRESHOLD_N,
            ),
            (
                CONTACT_REFERENCE_SPEED_PARAMETER,
                DEFAULT_CONTACT_REFERENCE_SPEED_MPS,
            ),
            (
                CONTACT_REFERENCE_THRESHOLD_PARAMETER,
                DEFAULT_CONTACT_REFERENCE_THRESHOLD_N,
            ),
            (
                CONTACT_MAXIMUM_THRESHOLD_PARAMETER,
                DEFAULT_CONTACT_MAXIMUM_THRESHOLD_N,
            ),
            (
                CONTACT_CONSECUTIVE_SAMPLES_PARAMETER,
                DEFAULT_CONTACT_CONSECUTIVE_SAMPLES,
            ),
        )
        values = {}
        for name, default in parameters:
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            values[name] = node.get_parameter(name).value

        return cls(
            minimum_threshold_n=float(
                values[CONTACT_MINIMUM_THRESHOLD_PARAMETER]
            ),
            reference_speed_mps=float(
                values[CONTACT_REFERENCE_SPEED_PARAMETER]
            ),
            reference_threshold_n=float(
                values[CONTACT_REFERENCE_THRESHOLD_PARAMETER]
            ),
            maximum_threshold_n=float(
                values[CONTACT_MAXIMUM_THRESHOLD_PARAMETER]
            ),
            consecutive_samples=int(
                values[CONTACT_CONSECUTIVE_SAMPLES_PARAMETER]
            ),
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
    "CONTACT_CONSECUTIVE_SAMPLES_PARAMETER",
    "CONTACT_MAXIMUM_THRESHOLD_PARAMETER",
    "CONTACT_MINIMUM_THRESHOLD_PARAMETER",
    "CONTACT_REFERENCE_SPEED_PARAMETER",
    "CONTACT_REFERENCE_THRESHOLD_PARAMETER",
    "DEFAULT_CONTACT_CONSECUTIVE_SAMPLES",
    "DEFAULT_CONTACT_MAXIMUM_THRESHOLD_N",
    "DEFAULT_CONTACT_MINIMUM_THRESHOLD_N",
    "DEFAULT_CONTACT_REFERENCE_SPEED_MPS",
    "DEFAULT_CONTACT_REFERENCE_THRESHOLD_N",
    "SpeedAwareForceContactPolicy",
]
