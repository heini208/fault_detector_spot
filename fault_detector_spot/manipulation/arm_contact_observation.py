"""Immutable shadow telemetry for one guarded arm force sample."""

from dataclasses import dataclass
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]
JointVector = Tuple[float, ...]
JointNames = Tuple[str, ...]


@dataclass(frozen=True)
class ArmContactObservation:
    """Synchronized evidence captured while a guarded arm move is active."""

    movement_sequence: int
    observed_at: float
    elapsed_sec: float
    phase: str
    planned_linear_speed_mps: float
    direction_frame: str
    movement_direction: Vector3
    force_received_at: float
    force_hand_n: Vector3
    baseline_force_hand_n: Vector3
    force_delta_n: Vector3
    opposing_force_delta_n: float
    total_force_delta_n: float
    hand_velocity_received_at: Optional[float]
    hand_linear_velocity_mps: Optional[Vector3]
    hand_angular_velocity_rad_s: Optional[Vector3]
    joint_state_received_at: Optional[float]
    joint_names: JointNames
    joint_positions_rad: Optional[JointVector]
    joint_velocities_rad_s: Optional[JointVector]
    joint_efforts_nm: Optional[JointVector]


__all__ = [
    "ArmContactObservation",
    "JointNames",
    "JointVector",
    "Vector3",
]
