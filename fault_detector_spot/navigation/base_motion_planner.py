"""Resolved movement plans for Spot base motion."""

from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.walking_profile import WalkingProfile


@dataclass(frozen=True)
class BaseMovementPlan:
    """Resolved planar base target and walking settings."""

    target: PoseStamped
    linear_speed_mps: float
    profile: WalkingProfile


__all__ = ["BaseMovementPlan"]
