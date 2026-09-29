"""Retry policy for measured Spot base endpoint errors."""

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True)
class BaseCorrectionConfig:
    """Configuration for bounded base endpoint correction."""

    maximum_attempts: int = 2
    minimum_progress_ratio: float = 0.10

    def __post_init__(self):
        if (
            isinstance(self.maximum_attempts, bool)
            or not isinstance(self.maximum_attempts, int)
            or self.maximum_attempts < 0
        ):
            raise ValueError(
                "Maximum correction attempts must be a non-negative integer"
            )
        if (
            not math.isfinite(self.minimum_progress_ratio)
            or not 0.0 < self.minimum_progress_ratio < 1.0
        ):
            raise ValueError(
                "Correction progress ratio must be between zero and one"
            )

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        maximum_attempts_key = "base.correction.maximum_attempts"
        minimum_progress_key = "base.correction.minimum_progress_ratio"

        if not node.has_parameter(maximum_attempts_key):
            node.declare_parameter(
                maximum_attempts_key,
                defaults.maximum_attempts,
            )
        if not node.has_parameter(minimum_progress_key):
            node.declare_parameter(
                minimum_progress_key,
                defaults.minimum_progress_ratio,
            )

        return cls(
            maximum_attempts=node.get_parameter(
                maximum_attempts_key
            ).value,
            minimum_progress_ratio=float(
                node.get_parameter(minimum_progress_key).value
            ),
        )


class BaseCorrectionDecision(Enum):
    """Action selected after failed endpoint verification."""

    RETRY_FROZEN_PLAN = "retry_frozen_plan"
    FAIL = "fail"


@dataclass(frozen=True)
class BaseCorrectionResult:
    """One correction-policy decision."""

    decision: BaseCorrectionDecision
    detail: str = ""
    attempt: int = 0


class BaseCorrectionPolicy:
    """Decide whether an inaccurate endpoint merits another frozen retry."""

    def __init__(self, config=None):
        self.config = config or BaseCorrectionConfig()
        if not isinstance(self.config, BaseCorrectionConfig):
            raise TypeError(
                "BaseCorrectionPolicy requires BaseCorrectionConfig"
            )
        self.reset()

    @property
    def attempts(self) -> int:
        return self._attempts

    def reset(self) -> None:
        self._attempts = 0
        self._previous_error_score = None

    def decide(
        self,
        error,
        position_tolerance_m: float,
        yaw_tolerance_rad: float,
    ) -> BaseCorrectionResult:
        if error is None:
            return BaseCorrectionResult(BaseCorrectionDecision.FAIL)
        if len(error) != 2:
            raise ValueError(
                "Base correction error must contain position and yaw"
            )

        position_error = self._non_negative(
            error[0],
            "Base correction position error",
        )
        yaw_error = self._non_negative(
            error[1],
            "Base correction yaw error",
        )
        position_tolerance = self._positive(
            position_tolerance_m,
            "Base correction position tolerance",
        )
        yaw_tolerance = self._positive(
            yaw_tolerance_rad,
            "Base correction yaw tolerance",
        )
        score = max(
            position_error / position_tolerance,
            yaw_error / yaw_tolerance,
        )

        if score <= 1.0:
            return BaseCorrectionResult(BaseCorrectionDecision.FAIL)

        if self._attempts >= self.config.maximum_attempts:
            return BaseCorrectionResult(
                BaseCorrectionDecision.FAIL,
                "correction attempt limit reached",
                self._attempts,
            )

        previous = self._previous_error_score
        if (
            previous is not None
            and score
            > previous * (1.0 - self.config.minimum_progress_ratio)
        ):
            return BaseCorrectionResult(
                BaseCorrectionDecision.FAIL,
                "corrections made insufficient progress",
                self._attempts,
            )

        self._previous_error_score = score
        self._attempts += 1
        return BaseCorrectionResult(
            BaseCorrectionDecision.RETRY_FROZEN_PLAN,
            attempt=self._attempts,
        )

    @staticmethod
    def _positive(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{label} must be positive and finite")
        return normalized

    @staticmethod
    def _non_negative(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(f"{label} must be non-negative and finite")
        return normalized


__all__ = [
    "BaseCorrectionConfig",
    "BaseCorrectionDecision",
    "BaseCorrectionPolicy",
    "BaseCorrectionResult",
]
