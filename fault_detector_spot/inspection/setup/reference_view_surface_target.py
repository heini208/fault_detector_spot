"""Surface-target model for probe setup geometry."""

from dataclasses import dataclass
from typing import Tuple

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_to_rpy as scipy_quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


@dataclass(frozen=True)
class ReferenceSurfaceTarget:
    """Nominal sensor-tip target and aligned pre-approach poses."""

    surface_point_object: Vector3Data
    outward_direction_object: Vector3Data
    target_pose_object: PoseData
    aligned_preapproach_pose_object: PoseData
    target_surface_distance_m: float
    aligned_preapproach_distance_m: float
    direction_source: str


def quaternion_to_rpy(
    quaternion: QuaternionData,
) -> Tuple[float, float, float]:
    """Convert one quaternion into roll, pitch, and yaw."""
    return scipy_quaternion_to_rpy(quaternion)


__all__ = [
    "ReferenceSurfaceTarget",
    "quaternion_to_rpy",
]
