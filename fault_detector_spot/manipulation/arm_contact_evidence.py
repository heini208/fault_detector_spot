"""Shadow evidence classification for guarded arm contact telemetry."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)


SHADOW_OFF_AXIS_SPEED_PARAMETER = (
    "arm.contact.shadow.off_axis_speed_threshold_mps"
)
DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS = 0.040


class ShadowContactClassification(Enum):
    """Non-authoritative interpretation of guarded arm contact evidence."""

    NONE = "none"
    FORCE_CANDIDATE = "force_candidate"
    LIKELY_EXTERNAL_CONTACT = "likely_external_contact"
    LIKELY_SELF_MOTION = "likely_self_motion"


@dataclass(frozen=True)
class ArmContactEvidence:
    """Derived shadow evidence for one synchronized telemetry sample."""

    classification: ShadowContactClassification
    force_threshold_n: float
    force_threshold_exceeded: bool
    force_candidate_count: int
    required_consecutive_samples: int
    parallel_hand_speed_mps: Optional[float]
    off_axis_hand_speed_mps: Optional[float]
    off_axis_speed_ratio: Optional[float]
    max_joint_velocity_rad_s: Optional[float]
    joint_effort_rate_max_nm_s: Optional[float]
    position_progress_rate_mps: Optional[float]
    off_axis_speed_threshold_mps: float


class ArmContactEvidenceAnalyzer:
    """Classify contact evidence without controlling the active movement."""

    def __init__(
        self,
        force_contact_policy=None,
        off_axis_speed_threshold_mps: float = (
            DEFAULT_SHADOW_OFF_AXIS_SPEED_THRESHOLD_MPS
        ),
    ):
        self.force_contact_policy = (
            force_contact_policy
            if force_contact_policy is not None
            else SpeedAwareForceContactPolicy()
        )
        if not isinstance(
            self.force_contact_policy,
            SpeedAwareForceContactPolicy,
        ):
            raise TypeError(
                "Shadow contact evidence requires a speed-aware force policy"
            )
        self.off_axis_speed_threshold_mps = self._positive(
            off_axis_speed_threshold_mps,
            "Shadow off-axis hand speed threshold",
        )
        self._movement_sequence = None
        self._force_candidate_count = 0
        self._previous_joint_received_at = None
        self._previous_joint_efforts_nm = None
        self._previous_observed_at = None
        self._previous_position_error_m = None

    @classmethod
    def from_node(cls, node):
        """Create the shadow analyzer from the current ROS parameters."""
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
            force_contact_policy=(
                SpeedAwareForceContactPolicy.from_node(node)
            ),
            off_axis_speed_threshold_mps=float(
                node.get_parameter(
                    SHADOW_OFF_AXIS_SPEED_PARAMETER
                ).value
            ),
        )

    def begin_movement(self, movement_sequence: int) -> None:
        """Reset stateful evidence for a newly started guarded movement."""
        self._movement_sequence = int(movement_sequence)
        self._force_candidate_count = 0
        self._previous_joint_received_at = None
        self._previous_joint_efforts_nm = None
        self._previous_observed_at = None
        self._previous_position_error_m = None

    def analyze(
        self,
        *,
        movement_sequence: int,
        observed_at: float,
        planned_linear_speed_mps: float,
        movement_direction: Tuple[float, float, float],
        opposing_force_delta_n: float,
        hand_linear_velocity_mps=None,
        joint_state_received_at=None,
        joint_velocities_rad_s=None,
        joint_efforts_nm=None,
        position_error_m=None,
    ) -> ArmContactEvidence:
        """Return one shadow classification and its supporting metrics."""
        sequence = int(movement_sequence)
        if self._movement_sequence != sequence:
            self.begin_movement(sequence)

        threshold_n = self.force_contact_policy.threshold_for(
            planned_linear_speed_mps
        )
        threshold_exceeded = (
            float(opposing_force_delta_n) >= threshold_n
        )
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
        max_joint_velocity = self._max_abs(joint_velocities_rad_s)
        effort_rate = self._joint_effort_rate(
            joint_state_received_at,
            joint_efforts_nm,
        )
        progress_rate = self._position_progress_rate(
            observed_at,
            position_error_m,
        )

        classification = ShadowContactClassification.NONE
        if threshold_exceeded:
            classification = ShadowContactClassification.FORCE_CANDIDATE
            sustained = (
                self._force_candidate_count
                >= self.force_contact_policy.consecutive_samples
            )
            if sustained and off_axis_speed is not None:
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
            required_consecutive_samples=(
                self.force_contact_policy.consecutive_samples
            ),
            parallel_hand_speed_mps=parallel_speed,
            off_axis_hand_speed_mps=off_axis_speed,
            off_axis_speed_ratio=off_axis_ratio,
            max_joint_velocity_rad_s=max_joint_velocity,
            joint_effort_rate_max_nm_s=effort_rate,
            position_progress_rate_mps=progress_rate,
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
    def _max_abs(values):
        if values is None:
            return None
        normalized = tuple(float(value) for value in values)
        if not normalized or not all(
            math.isfinite(value) for value in normalized
        ):
            return None
        return max(abs(value) for value in normalized)

    def _joint_effort_rate(self, received_at, efforts):
        rate = None
        if received_at is not None and efforts is not None:
            timestamp = float(received_at)
            current = tuple(float(value) for value in efforts)
            previous_time = self._previous_joint_received_at
            previous = self._previous_joint_efforts_nm
            if (
                math.isfinite(timestamp)
                and current
                and all(math.isfinite(value) for value in current)
            ):
                if (
                    previous_time is not None
                    and previous is not None
                    and len(previous) == len(current)
                    and timestamp > previous_time + 1e-12
                ):
                    dt = timestamp - previous_time
                    rate = max(
                        abs(current_value - previous_value) / dt
                        for current_value, previous_value in zip(
                            current,
                            previous,
                        )
                    )
                if (
                    previous_time is None
                    or timestamp > previous_time + 1e-12
                ):
                    self._previous_joint_received_at = timestamp
                    self._previous_joint_efforts_nm = current
        return rate

    def _position_progress_rate(self, observed_at, position_error_m):
        rate = None
        if position_error_m is not None:
            timestamp = float(observed_at)
            error = float(position_error_m)
            previous_time = self._previous_observed_at
            previous_error = self._previous_position_error_m
            if math.isfinite(timestamp) and math.isfinite(error):
                if (
                    previous_time is not None
                    and previous_error is not None
                    and timestamp > previous_time + 1e-12
                ):
                    rate = (
                        previous_error - error
                    ) / (timestamp - previous_time)
                self._previous_observed_at = timestamp
                self._previous_position_error_m = error
        return rate

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
