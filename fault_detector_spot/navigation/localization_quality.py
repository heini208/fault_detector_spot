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


@dataclass(frozen=True)
class LocalizationAssessment:
    """Separate measured uncertainty from missing or invalid evidence."""

    detail: str = ""
    uncertain: bool = False
    not_localized: bool = False

    @property
    def healthy(self):
        return not self.detail


def assess_localization_quality(
    message, now_ns: int, config: LocalizationQualityConfig,
) -> LocalizationAssessment:
    """Classify a fresh estimate without treating a data fault as uncertainty.

    This checks the planar marginal covariance, including cross terms. A
    positive covariance is required: RTAB-Map's unknown (9999) covariance and
    unset all-zero messages must never count as evidence of good localization.
    """
    if message is None:
        return LocalizationAssessment("No localization estimate received")
    try:
        if message.header.frame_id != "map":
            return LocalizationAssessment("Localization estimate is not in the map frame")
        stamp = message.header.stamp
        if not 0 <= stamp.nanosec < 1_000_000_000:
            return LocalizationAssessment("Localization timestamp is invalid")
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0 or not math.isfinite(now_ns):
            return LocalizationAssessment("Localization timestamp is invalid")
        age_ns = now_ns - stamp_ns
        if age_ns < 0:
            return LocalizationAssessment("Localization estimate is future-dated")
        if age_ns > config.maximum_age_sec * 1_000_000_000:
            return LocalizationAssessment("Localization estimate is stale")
        pose_to_matrix(message.pose.pose)
        covariance = np.asarray(message.pose.covariance, dtype=float)
        if covariance.shape != (36,):
            return LocalizationAssessment("Localization covariance must contain 36 values")
        planar = covariance.reshape(6, 6)[np.ix_((0, 1, 5), (0, 1, 5))]
        if not np.all(np.isfinite(planar)):
            return LocalizationAssessment("Localization covariance is non-finite")
        diagonal = np.diag(planar)
        # CoreWrapper publishes this exact planar sentinel when it has no
        # localization covariance. It is an explicit not-localized report,
        # unlike a missing estimate or an arbitrary malformed covariance.
        if np.array_equal(planar, np.eye(3) * 9999.0):
            return LocalizationAssessment(
                "RTAB-Map reports not localized (unknown covariance)",
                uncertain=True, not_localized=True,
            )
        if np.any(diagonal <= 0) or np.any(diagonal >= 9999):
            return LocalizationAssessment("Localization covariance is unknown or invalid")
        if not np.allclose(planar, planar.T, rtol=1e-6, atol=1e-12):
            return LocalizationAssessment("Localization covariance is not symmetric")
        # Use the symmetric matrix after checking numerical round-off only.
        planar = (planar + planar.T) * 0.5
        if np.linalg.eigvalsh(planar)[0] < -1e-12:
            return LocalizationAssessment("Localization covariance is not positive semidefinite")
        position_std = math.sqrt(np.linalg.eigvalsh(planar[:2, :2])[-1])
        yaw_std = math.sqrt(planar[2, 2])
        if position_std > config.maximum_position_std_m:
            return LocalizationAssessment("Localization position uncertainty is too high", uncertain=True)
        if yaw_std > config.maximum_yaw_std_rad:
            return LocalizationAssessment("Localization yaw uncertainty is too high", uncertain=True)
    except (AttributeError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
        return LocalizationAssessment("Localization estimate is malformed")
    return LocalizationAssessment()


def localization_quality_error(message, now_ns: int, config: LocalizationQualityConfig) -> str:
    """Explain why a pose cannot qualify for authoring or refinement."""
    return assess_localization_quality(message, now_ns, config).detail


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


def accepted_lidar_localization(info) -> bool:
    """Whether this result includes an accepted LiDAR-to-map registration.

    RTAB-Map publishes both proximity ICP counters in each extended result,
    including zero for the unused path. Counters alone are insufficient: they
    remain positive if subsequent optimization rejects the added links. The
    proximity ID and rejection flag must therefore also confirm acceptance.
    NeighborLinkRefining ICP statistics describe odometry, not a map match.

    Callers still need matching capture timestamps, localization quality and
    seed provenance checks; an accepted ICP match is not independent truth.
    """
    if not accepted_localization(info):
        return False
    try:
        if info.proximity_detection_id <= 0:
            return False
        keys = list(info.stats_keys)
        counters = []
        for key in (
            "Proximity/Space_detections_added_icp_global/",
            "Proximity/Space_detections_added_icp_multi/",
        ):
            if keys.count(key) != 1:
                return False
            value = info.stats_values[keys.index(key)]
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                return False
            counters.append(value)
        return any(value > 0 for value in counters)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


def lidar_localization_result(info) -> bool | None:
    """Distinguish accepted LiDAR registration from explicit failure or no evidence.

    False requires a complete valid result with no accepted map registration.
    Missing statistics and an accepted registration without verified LiDAR ICP
    are unknown, so neither can justify landmark recovery. Callers must also
    validate the frame, matching capture timestamp, and freshness.
    """
    if info is None:
        return None
    try:
        identifiers = (info.loop_closure_id, info.proximity_detection_id)
        if not all(type(value) is int and value >= 0 for value in identifiers):
            return None
        keys = list(info.stats_keys)
        if len(keys) != len(info.stats_values):
            return None
        required = (
            "Loop/RejectedHypothesis/",
            "Proximity/Space_detections_added_icp_global/",
            "Proximity/Space_detections_added_icp_multi/",
        )
        values = []
        for key in required:
            if keys.count(key) != 1:
                return None
            value = info.stats_values[keys.index(key)]
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                return None
            values.append(value)
        if values[0] not in (0.0, 1.0):
            return None
        if accepted_lidar_localization(info):
            return True
        if accepted_localization(info):
            return None
        return False
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
