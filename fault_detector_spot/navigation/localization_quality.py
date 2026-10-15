"""Shared validation of map localization estimates and registration evidence."""

from dataclasses import dataclass, fields
import math

import numpy as np

from fault_detector_spot.shared.geometry.transforms import pose_to_matrix
from fault_detector_spot.shared.persistence.basic_settings import parameter_override


@dataclass(frozen=True)
class LocalizationQualityConfig:
    maximum_age_sec: float = 1.5
    maximum_position_std_m: float = 0.30
    maximum_yaw_std_rad: float = math.radians(10.0)

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"Localization quality {field.name} must be positive and finite"
                )

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(cls):
            key = f"navigation.localization_quality.{field.name}"
            if not node.has_parameter(key):
                node.declare_parameter(key, getattr(defaults, field.name))
            values[field.name] = float(
                parameter_override(key, node.get_parameter(key).value)
            )
        return cls(**values)


def localization_quality_error(
    message, now_ns: int, config: LocalizationQualityConfig,
) -> str:
    """Return an explanation for an unusable estimate, or an empty string.

    This checks the planar marginal covariance, including cross terms. A
    positive covariance is required: RTAB-Map's unknown (9999) covariance and
    unset all-zero messages must never count as evidence of good localization.
    """
    if message is None:
        return "No localization estimate received"
    try:
        if message.header.frame_id != "map":
            return "Localization estimate is not in the map frame"
        stamp = message.header.stamp
        if not 0 <= stamp.nanosec < 1_000_000_000:
            return "Localization timestamp is invalid"
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0 or not math.isfinite(now_ns):
            return "Localization timestamp is invalid"
        age_ns = now_ns - stamp_ns
        if age_ns < 0:
            return "Localization estimate is future-dated"
        if age_ns > config.maximum_age_sec * 1_000_000_000:
            return "Localization estimate is stale"
        pose_to_matrix(message.pose.pose)
        covariance = np.asarray(message.pose.covariance, dtype=float)
        if covariance.shape != (36,):
            return "Localization covariance must contain 36 values"
        planar = covariance.reshape(6, 6)[np.ix_((0, 1, 5), (0, 1, 5))]
        if not np.all(np.isfinite(planar)):
            return "Localization covariance is non-finite"
        diagonal = np.diag(planar)
        if np.any(diagonal <= 0) or np.any(diagonal >= 9999):
            return "Localization covariance is unknown or invalid"
        if not np.allclose(planar, planar.T, rtol=1e-6, atol=1e-12):
            return "Localization covariance is not symmetric"
        # Use the symmetric matrix after checking numerical round-off only.
        planar = (planar + planar.T) * 0.5
        if np.linalg.eigvalsh(planar)[0] < -1e-12:
            return "Localization covariance is not positive semidefinite"
        position_std = math.sqrt(np.linalg.eigvalsh(planar[:2, :2])[-1])
        yaw_std = math.sqrt(planar[2, 2])
        if position_std > config.maximum_position_std_m:
            return "Localization position uncertainty is too high"
        if yaw_std > config.maximum_yaw_std_rad:
            return "Localization yaw uncertainty is too high"
    except (AttributeError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
        return "Localization estimate is malformed"
    return ""


def accepted_localization(info) -> bool:
    """Whether RTAB-Map reports an accepted map registration in this result.

    Consumers must also check frame and matching observation timestamps. Merely
    retaining a previous covariance or receiving an initial pose is not proof
    that a new map registration succeeded.
    """
    if info is None:
        return False
    try:
        identifiers = (info.loop_closure_id, info.proximity_detection_id)
        if not all(type(value) is int and value >= 0 for value in identifiers):
            return False
        if not any(value > 0 for value in identifiers):
            return False
        if len(info.stats_keys) != len(info.stats_values):
            return False
        rejection_key = "Loop/RejectedHypothesis/"
        if list(info.stats_keys).count(rejection_key) != 1:
            return False
        index = list(info.stats_keys).index(rejection_key)
        rejection = float(info.stats_values[index])
        return math.isfinite(rejection) and rejection == 0.0
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
