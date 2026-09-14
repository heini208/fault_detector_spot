"""Contact evidence classification for guarded arm motion."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple


SHADOW_OFF_AXIS_SPEED_PARAMETER = (
    "arm.contact.shadow.off_axis_speed_threshold_mps"
)
DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS = 0.040


class ShadowContactClassification(Enum):
    """Interpret one sustained force candidate from measured hand motion."""

    NONE = "none"
    FORCE_CANDIDATE = "force_candidate"
    LIKELY_EXTERNAL_CONTACT = "likely_external_contact"
    LIKELY_SELF_MOTION = "likely_self_motion"


@dataclass(frozen=True)
class ArmContactEvidence:
    """Derived contact evidence for one guarded force sample."""

    classification: ShadowContactClassification
    force_threshold_n: float
    force_threshold_exceeded: bool
    force_candidate_count: int
    required_consecutive_samples: int
    parallel_hand_speed_mps: Optional[float]
    off_axis_hand_speed_mps: Optional[float]
    off_axis_speed_ratio: Optional[float]
    off_axis_speed_threshold_mps: float


class ArmContactEvidenceAnalyzer:
    """Recognize the validated self-motion signature without owning policy."""

    def __init__(
        self,
        off_axis_speed_threshold_mps: float = (
            DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS
        ),
    ):
        self.off_axis_speed_threshold_mps = self._positive(
            off_axis_speed_threshold_mps,
            "Contact off-axis hand speed threshold",
        )
        self._force_candidate_count = 0

    @classmethod
    def from_node(cls, node):
        if node is None:
            raise RuntimeError(
                "ArmContactEvidenceAnalyzer requires a ROS node"
            )
        if not node.has_parameter(SHADOW_OFF_AXIS_SPEED_PARAMETER):
            node.declare_parameter(
                SHADOW_OFF_AXIS_SPEED_PARAMETER,
                DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS,
            )
        return cls(
            off_axis_speed_threshold_mps=float(
                node.get_parameter(
                    SHADOW_OFF_AXIS_SPEED_PARAMETER
                ).value
            ),
        )

    def begin_movement(self) -> None:
        self._force_candidate_count = 0

    def analyze(
        self,
        *,
        force_threshold_n: float,
        required_consecutive_samples: int,
        movement_direction: Tuple[float, float, float],
        opposing_force_delta_n: float,
        hand_linear_velocity_mps=None,
    ) -> ArmContactEvidence:
        threshold_n = self._positive(
            force_threshold_n,
            "Authoritative force threshold",
        )
        if (
            isinstance(required_consecutive_samples, bool)
            or not isinstance(required_consecutive_samples, int)
            or required_consecutive_samples < 1
        ):
            raise ValueError(
                "Required consecutive contact samples must be positive"
            )

        opposing_force = float(opposing_force_delta_n)
        if not math.isfinite(opposing_force):
            raise ValueError("Opposing force delta must be finite")

        threshold_exceeded = opposing_force >= threshold_n
        if threshold_exceeded:
            self._force_candidate_count += 1
        else:
            self._force_candidate_count = 0

        parallel_speed, off_axis_speed, off_axis_ratio = (
            self._hand_velocity_metrics(
                hand_linear_velocity_mps,
                movement_direction,
            )
        )

        classification = ShadowContactClassification.NONE
        if threshold_exceeded:
            classification = ShadowContactClassification.FORCE_CANDIDATE
            if (
                self._force_candidate_count
                >= required_consecutive_samples
                and off_axis_speed is not None
            ):
                if (
                    off_axis_speed
                    >= self.off_axis_speed_threshold_mps
                ):
                    classification = (
                        ShadowContactClassification.LIKELY_SELF_MOTION
                    )
                else:
                    classification = (
                        ShadowContactClassification.LIKELY_EXTERNAL_CONTACT
                    )

        return ArmContactEvidence(
            classification=classification,
            force_threshold_n=threshold_n,
            force_threshold_exceeded=threshold_exceeded,
            force_candidate_count=self._force_candidate_count,
            required_consecutive_samples=required_consecutive_samples,
            parallel_hand_speed_mps=parallel_speed,
            off_axis_hand_speed_mps=off_axis_speed,
            off_axis_speed_ratio=off_axis_ratio,
            off_axis_speed_threshold_mps=(
                self.off_axis_speed_threshold_mps
            ),
        )

    @staticmethod
    def _hand_velocity_metrics(velocity, direction):
        if velocity is None:
            return None, None, None
        values = tuple(float(value) for value in velocity)
        axes = tuple(float(value) for value in direction)
        if len(values) != 3 or len(axes) != 3:
            return None, None, None
        if not all(math.isfinite(value) for value in values + axes):
            return None, None, None

        direction_norm = math.sqrt(
            sum(value * value for value in axes)
        )
        if direction_norm <= 1e-12:
            return None, None, None
        unit = tuple(value / direction_norm for value in axes)
        parallel = sum(
            value * axis for value, axis in zip(values, unit)
        )
        off_axis_vector = tuple(
            value - parallel * axis
            for value, axis in zip(values, unit)
        )
        off_axis = math.sqrt(
            sum(value * value for value in off_axis_vector)
        )
        total = math.sqrt(sum(value * value for value in values))
        ratio = None if total <= 1e-12 else off_axis / total
        return parallel, off_axis, ratio

    @staticmethod
    def _positive(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized


__all__ = [
    "ArmContactEvidence",
    "ArmContactEvidenceAnalyzer",
    "DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS",
    "SHADOW_OFF_AXIS_SPEED_PARAMETER",
    "ShadowContactClassification",
]
