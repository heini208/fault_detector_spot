"""Speed-calibrated force threshold selection for guarded arm motion."""

from dataclasses import dataclass
import math


CONTACT_CALIBRATION_SPEEDS_PARAMETER = (
    "arm.contact.calibration_linear_speeds_mps"
)
CONTACT_CALIBRATION_THRESHOLDS_PARAMETER = (
    "arm.contact.calibration_force_delta_thresholds_n"
)
CONTACT_CONSECUTIVE_SAMPLES_PARAMETER = (
    "arm.contact.consecutive_samples"
)

DEFAULT_CONTACT_CALIBRATION_SPEEDS_MPS = (0.005,)
DEFAULT_CONTACT_CALIBRATION_THRESHOLDS_N = (5.0,)
DEFAULT_CONTACT_CONSECUTIVE_SAMPLES = 2


class UncalibratedForceSpeed(ValueError):
    """Raised when guarded motion requests an uncalibrated speed."""


@dataclass(frozen=True)
class ForceThresholdCalibration:
    linear_speed_mps: float
    force_delta_threshold_n: float

    def __post_init__(self):
        if (
            not math.isfinite(float(self.linear_speed_mps))
            or float(self.linear_speed_mps) <= 0.0
        ):
            raise ValueError(
                "Force calibration speed must be positive and finite"
            )
        if (
            not math.isfinite(float(self.force_delta_threshold_n))
            or float(self.force_delta_threshold_n) <= 0.0
        ):
            raise ValueError(
                "Force calibration threshold must be positive and finite"
            )


class SpeedAwareForceContactPolicy:
    """Interpolate only between measured speed/threshold calibration points."""

    def __init__(
        self,
        calibrations,
        consecutive_samples: int = DEFAULT_CONTACT_CONSECUTIVE_SAMPLES,
        speed_tolerance_mps: float = 1e-6,
    ):
        values = tuple(calibrations)
        if not values:
            raise ValueError(
                "Force contact policy requires calibration points"
            )
        if isinstance(consecutive_samples, bool) or int(consecutive_samples) < 1:
            raise ValueError(
                "Force contact consecutive sample count must be positive"
            )
        tolerance = float(speed_tolerance_mps)
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError(
                "Force calibration speed tolerance must be positive"
            )

        normalized = []
        for value in values:
            if not isinstance(value, ForceThresholdCalibration):
                raise TypeError(
                    "Force contact policy requires "
                    "ForceThresholdCalibration values"
                )
            normalized.append(value)
        normalized.sort(key=lambda value: value.linear_speed_mps)

        speeds = [value.linear_speed_mps for value in normalized]
        if any(
            abs(second - first) <= tolerance
            for first, second in zip(speeds, speeds[1:])
        ):
            raise ValueError(
                "Force calibration speeds must be unique"
            )

        self.calibrations = tuple(normalized)
        self.consecutive_samples = int(consecutive_samples)
        self.speed_tolerance_mps = tolerance

    @classmethod
    def from_node(cls, node):
        if node is None:
            raise RuntimeError(
                "SpeedAwareForceContactPolicy requires a ROS node"
            )

        defaults = (
            (
                CONTACT_CALIBRATION_SPEEDS_PARAMETER,
                list(DEFAULT_CONTACT_CALIBRATION_SPEEDS_MPS),
            ),
            (
                CONTACT_CALIBRATION_THRESHOLDS_PARAMETER,
                list(DEFAULT_CONTACT_CALIBRATION_THRESHOLDS_N),
            ),
            (
                CONTACT_CONSECUTIVE_SAMPLES_PARAMETER,
                DEFAULT_CONTACT_CONSECUTIVE_SAMPLES,
            ),
        )
        values = {}
        for name, default in defaults:
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            values[name] = node.get_parameter(name).value

        speeds = tuple(
            float(value)
            for value in values[CONTACT_CALIBRATION_SPEEDS_PARAMETER]
        )
        thresholds = tuple(
            float(value)
            for value in values[CONTACT_CALIBRATION_THRESHOLDS_PARAMETER]
        )
        if len(speeds) != len(thresholds):
            raise ValueError(
                "Force calibration speed and threshold arrays must "
                "have equal length"
            )

        return cls(
            calibrations=tuple(
                ForceThresholdCalibration(speed, threshold)
                for speed, threshold in zip(speeds, thresholds)
            ),
            consecutive_samples=int(
                values[CONTACT_CONSECUTIVE_SAMPLES_PARAMETER]
            ),
        )

    def threshold_for(self, linear_speed_mps: float) -> float:
        """Return a calibrated threshold without extrapolating."""
        speed = float(linear_speed_mps)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(
                "Guarded arm speed must be positive and finite"
            )

        for calibration in self.calibrations:
            if (
                abs(calibration.linear_speed_mps - speed)
                <= self.speed_tolerance_mps
            ):
                return calibration.force_delta_threshold_n

        lower = None
        upper = None
        for calibration in self.calibrations:
            if calibration.linear_speed_mps < speed:
                lower = calibration
                continue
            if calibration.linear_speed_mps > speed:
                upper = calibration
                break

        if lower is None or upper is None:
            available = ", ".join(
                f"{value.linear_speed_mps:.4f}"
                for value in self.calibrations
            )
            raise UncalibratedForceSpeed(
                f"No force threshold is calibrated for {speed:.4f} m/s. "
                f"Calibrated speed range/points: {available} m/s"
            )

        fraction = (
            (speed - lower.linear_speed_mps)
            / (upper.linear_speed_mps - lower.linear_speed_mps)
        )
        return (
            lower.force_delta_threshold_n
            + fraction
            * (
                upper.force_delta_threshold_n
                - lower.force_delta_threshold_n
            )
        )


__all__ = [
    "CONTACT_CALIBRATION_SPEEDS_PARAMETER",
    "CONTACT_CALIBRATION_THRESHOLDS_PARAMETER",
    "CONTACT_CONSECUTIVE_SAMPLES_PARAMETER",
    "DEFAULT_CONTACT_CALIBRATION_SPEEDS_MPS",
    "DEFAULT_CONTACT_CALIBRATION_THRESHOLDS_N",
    "DEFAULT_CONTACT_CONSECUTIVE_SAMPLES",
    "ForceThresholdCalibration",
    "SpeedAwareForceContactPolicy",
    "UncalibratedForceSpeed",
]
