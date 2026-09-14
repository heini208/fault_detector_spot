"""Immutable shadow telemetry for one guarded arm force sample."""

from dataclasses import dataclass
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
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
    initial_translation_distance_m: float
    current_hand_position_m: Optional[Vector3]
    current_hand_orientation_xyzw: Optional[Quaternion]
    target_hand_position_m: Vector3
    target_hand_orientation_xyzw: Quaternion
    position_error_m: Optional[float]
    rotation_error_rad: Optional[float]
    forward_progress_m: Optional[float]
    off_axis_displacement_m: Optional[float]
    translation_progress_fraction: Optional[float]
    position_progress_rate_mps: Optional[float]
    force_received_at: float
    force_hand_n: Vector3
    baseline_force_hand_n: Vector3
    force_delta_n: Vector3
    opposing_force_delta_n: float
    total_force_delta_n: float
    shadow_force_threshold_n: float
    shadow_force_threshold_exceeded: bool
    shadow_force_candidate_count: int
    shadow_required_consecutive_samples: int
    shadow_classification: str
    shadow_off_axis_speed_threshold_mps: float
    hand_velocity_received_at: Optional[float]
    hand_linear_velocity_mps: Optional[Vector3]
    hand_angular_velocity_rad_s: Optional[Vector3]
    parallel_hand_speed_mps: Optional[float]
    off_axis_hand_speed_mps: Optional[float]
    off_axis_speed_ratio: Optional[float]
    joint_state_received_at: Optional[float]
    joint_names: JointNames
    joint_positions_rad: Optional[JointVector]
    joint_velocities_rad_s: Optional[JointVector]
    joint_efforts_nm: Optional[JointVector]
    max_joint_velocity_rad_s: Optional[float]
    joint_effort_rate_max_nm_s: Optional[float]


__all__ = [
    "ArmContactObservation",
    "JointNames",
    "JointVector",
    "Quaternion",
    "Vector3",
]
