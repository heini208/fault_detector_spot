"""Transient probe-point setup poses and frame conversion helpers."""

import math
from dataclasses import dataclass

import numpy as np

from .models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from .surface_distance_validation import validate_surface_distance_pair
from .reference_view_surface_target import ReferenceSurfaceTarget


@dataclass(frozen=True)
class ReferenceProbeSetup:
    """User-adjustable transient poses for one selected surface point."""

    surface_target: ReferenceSurfaceTarget
    safe_approach_pose_object: PoseData
    aligned_preapproach_pose_object: PoseData
    probe_pose_object: PoseData
    safe_approach_approved: bool = False
    surface_alignment_approved: bool = False
    probe_pose_approved: bool = False


def initialize_reference_probe_setup(
    surface_target: ReferenceSurfaceTarget,
) -> ReferenceProbeSetup:
    """Create transient setup state from calculated surface geometry."""
    _validate_surface_target(surface_target)
    return ReferenceProbeSetup(
        surface_target=surface_target,
        safe_approach_pose_object=(
            surface_target.aligned_preapproach_pose_object
        ),
        aligned_preapproach_pose_object=(
            surface_target.aligned_preapproach_pose_object
        ),
        probe_pose_object=surface_target.target_pose_object,
    )


def approve_safe_approach_pose(
    setup: ReferenceProbeSetup,
    current_probe_pose_object: PoseData,
) -> ReferenceProbeSetup:
    """Use the current probe pose as the obstacle-safe approach pose."""
    _validate_setup(setup)
    current_probe_pose_object.validate()
    return ReferenceProbeSetup(
        surface_target=setup.surface_target,
        safe_approach_pose_object=current_probe_pose_object,
        aligned_preapproach_pose_object=(
            setup.aligned_preapproach_pose_object
        ),
        probe_pose_object=setup.probe_pose_object,
        safe_approach_approved=True,
        surface_alignment_approved=setup.surface_alignment_approved,
        probe_pose_approved=setup.probe_pose_approved,
    )


def approve_surface_alignment_pose(
    setup: ReferenceProbeSetup,
    current_probe_pose_object: PoseData,
) -> ReferenceProbeSetup:
    """Use the current aligned pose and rebuild the probe pose."""
    _validate_setup(setup)
    current_probe_pose_object.validate()
    distance_delta = _distance_delta(setup.surface_target)
    inward = rotate_vector(
        current_probe_pose_object.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    probe_position = add_vectors(
        current_probe_pose_object.position,
        scale_vector(inward, distance_delta),
    )
    orientation = current_probe_pose_object.orientation
    return ReferenceProbeSetup(
        surface_target=setup.surface_target,
        safe_approach_pose_object=setup.safe_approach_pose_object,
        aligned_preapproach_pose_object=current_probe_pose_object,
        probe_pose_object=PoseData(
            position=probe_position,
            orientation=orientation,
        ),
        safe_approach_approved=setup.safe_approach_approved,
        surface_alignment_approved=True,
        probe_pose_approved=False,
    )


def approve_probe_pose(
    setup: ReferenceProbeSetup,
    current_probe_pose_object: PoseData,
) -> ReferenceProbeSetup:
    """Use the current probe pose and rebuild its aligned pre-approach pose."""
    _validate_setup(setup)
    current_probe_pose_object.validate()
    aligned_pose = derive_aligned_preapproach_pose(
        current_probe_pose_object,
        setup.surface_target.target_surface_distance_m,
        setup.surface_target.aligned_preapproach_distance_m,
    )
    return ReferenceProbeSetup(
        surface_target=setup.surface_target,
        safe_approach_pose_object=setup.safe_approach_pose_object,
        aligned_preapproach_pose_object=aligned_pose,
        probe_pose_object=current_probe_pose_object,
        safe_approach_approved=setup.safe_approach_approved,
        surface_alignment_approved=True,
        probe_pose_approved=True,
    )


def derive_aligned_preapproach_pose(
    probe_pose_object: PoseData,
    target_surface_distance_m: float,
    aligned_preapproach_distance_m: float,
) -> PoseData:
    """Derive the aligned pose from shared alignment and two distances."""
    probe_pose_object.validate()
    distance_delta = _validated_distance_delta(
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    )
    inward = rotate_vector(
        probe_pose_object.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )
    result = PoseData(
        position=subtract_vectors(
            probe_pose_object.position,
            scale_vector(inward, distance_delta),
        ),
        orientation=probe_pose_object.orientation,
    )
    result.validate()
    return result


def refine_probe_pose(
    current_probe_pose_object: PoseData,
    surface_orientation_object: QuaternionData,
    local_translation: Vector3Data = None,
    pitch_rad: float = 0.0,
    yaw_rad: float = 0.0,
) -> PoseData:
    """Translate in supplied axes and rotate about the sensor tip."""
    current_probe_pose_object.validate()
    surface_orientation_object.validate()
    translation = local_translation or Vector3Data.zero()
    translation.validate()
    for label, value in (("Pitch", pitch_rad), ("Yaw", yaw_rad)):
        if not math.isfinite(value):
            raise ValueError(f"{label} refinement must be finite")

    translation_object = rotate_vector(
        surface_orientation_object,
        translation,
    )
    local_rotation = _pitch_yaw_quaternion(pitch_rad, yaw_rad)
    result = PoseData(
        position=add_vectors(
            current_probe_pose_object.position,
            translation_object,
        ),
        orientation=multiply_quaternions(
            current_probe_pose_object.orientation,
            local_rotation,
        ),
    )
    result.validate()
    return result


def invalidate_probe_setup_approvals(
    setup: ReferenceProbeSetup,
    stage: str,
) -> ReferenceProbeSetup:
    """Invalidate only geometry changed by one refinement stage."""
    _validate_setup(setup)
    if stage == "approach":
        safe_approved = False
        alignment_approved = setup.surface_alignment_approved
        probe_approved = setup.probe_pose_approved
    elif stage == "alignment":
        safe_approved = setup.safe_approach_approved
        alignment_approved = False
        probe_approved = False
    elif stage == "probe":
        safe_approved = setup.safe_approach_approved
        alignment_approved = setup.surface_alignment_approved
        probe_approved = False
    else:
        raise ValueError(f"Unknown probe refinement stage: {stage}")
    return ReferenceProbeSetup(
        surface_target=setup.surface_target,
        safe_approach_pose_object=setup.safe_approach_pose_object,
        aligned_preapproach_pose_object=(
            setup.aligned_preapproach_pose_object
        ),
        probe_pose_object=setup.probe_pose_object,
        safe_approach_approved=safe_approved,
        surface_alignment_approved=alignment_approved,
        probe_pose_approved=probe_approved,
    )


def probe_pose_to_hand_pose(
    desired_probe_pose_parent: PoseData,
    hand_to_probe_pose: PoseData,
) -> PoseData:
    """Return the hand pose that places the probe frame at the desired pose."""
    desired_probe_pose_parent.validate()
    hand_to_probe_pose.validate()
    return compose_poses(
        desired_probe_pose_parent,
        inverse_pose(hand_to_probe_pose),
    )


def compose_poses(
    parent_to_child: PoseData,
    child_to_target: PoseData,
) -> PoseData:
    """Compose two poses expressed as parent-to-child transforms."""
    parent_to_child.validate()
    child_to_target.validate()
    rotated_position = rotate_vector(
        parent_to_child.orientation,
        child_to_target.position,
    )
    result = PoseData(
        position=add_vectors(
            parent_to_child.position,
            rotated_position,
        ),
        orientation=multiply_quaternions(
            parent_to_child.orientation,
            child_to_target.orientation,
        ),
    )
    result.validate()
    return result


def relative_pose(
    parent_to_reference: PoseData,
    parent_to_target: PoseData,
) -> PoseData:
    """Express a target pose in a reference frame."""
    return compose_poses(
        inverse_pose(parent_to_reference),
        parent_to_target,
    )


def inverse_pose(pose: PoseData) -> PoseData:
    """Invert one rigid pose."""
    pose.validate()
    inverse_orientation = QuaternionData(
        x=-pose.orientation.x,
        y=-pose.orientation.y,
        z=-pose.orientation.z,
        w=pose.orientation.w,
    )
    inverse_position = rotate_vector(
        inverse_orientation,
        Vector3Data(
            x=-pose.position.x,
            y=-pose.position.y,
            z=-pose.position.z,
        ),
    )
    result = PoseData(
        position=inverse_position,
        orientation=inverse_orientation,
    )
    result.validate()
    return result


def multiply_quaternions(
    first: QuaternionData,
    second: QuaternionData,
) -> QuaternionData:
    """Multiply two normalized quaternions."""
    first.validate()
    second.validate()
    x1, y1, z1, w1 = first.x, first.y, first.z, first.w
    x2, y2, z2, w2 = second.x, second.y, second.z, second.w
    values = np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Quaternion product cannot be normalized")
    values /= norm
    result = QuaternionData(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
        w=float(values[3]),
    )
    result.validate()
    return result


def rotate_vector(
    quaternion: QuaternionData,
    vector: Vector3Data,
) -> Vector3Data:
    """Rotate a vector by a normalized quaternion."""
    quaternion.validate()
    vector.validate()
    q = np.array(
        [quaternion.x, quaternion.y, quaternion.z],
        dtype=float,
    )
    value = np.array([vector.x, vector.y, vector.z], dtype=float)
    rotated = (
        value
        + 2.0 * quaternion.w * np.cross(q, value)
        + 2.0 * np.cross(q, np.cross(q, value))
    )
    return Vector3Data(
        x=float(rotated[0]),
        y=float(rotated[1]),
        z=float(rotated[2]),
    )


def add_vectors(first: Vector3Data, second: Vector3Data) -> Vector3Data:
    """Add two vectors."""
    first.validate()
    second.validate()
    return Vector3Data(
        x=first.x + second.x,
        y=first.y + second.y,
        z=first.z + second.z,
    )


def subtract_vectors(first: Vector3Data, second: Vector3Data) -> Vector3Data:
    """Subtract two vectors."""
    first.validate()
    second.validate()
    return Vector3Data(
        x=first.x - second.x,
        y=first.y - second.y,
        z=first.z - second.z,
    )


def scale_vector(vector: Vector3Data, factor: float) -> Vector3Data:
    """Scale one vector."""
    vector.validate()
    if not math.isfinite(factor):
        raise ValueError("Vector scale must be finite")
    return Vector3Data(
        x=vector.x * factor,
        y=vector.y * factor,
        z=vector.z * factor,
    )


def _pitch_yaw_quaternion(
    pitch_rad: float,
    yaw_rad: float,
) -> QuaternionData:
    half_pitch = pitch_rad * 0.5
    half_yaw = yaw_rad * 0.5
    pitch = QuaternionData(
        x=0.0,
        y=math.sin(half_pitch),
        z=0.0,
        w=math.cos(half_pitch),
    )
    yaw = QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )
    return multiply_quaternions(yaw, pitch)


def _distance_delta(surface_target: ReferenceSurfaceTarget) -> float:
    _validate_surface_target(surface_target)
    return _validated_distance_delta(
        surface_target.target_surface_distance_m,
        surface_target.aligned_preapproach_distance_m,
    )


def _validated_distance_delta(
    target_surface_distance_m: float,
    aligned_preapproach_distance_m: float,
) -> float:
    return validate_surface_distance_pair(
        target_surface_distance_m,
        aligned_preapproach_distance_m,
    )


def _validate_setup(setup: ReferenceProbeSetup) -> None:
    if setup is None:
        raise ValueError("No transient probe setup is available")
    _validate_surface_target(setup.surface_target)
    setup.safe_approach_pose_object.validate()
    setup.aligned_preapproach_pose_object.validate()
    setup.probe_pose_object.validate()


def _validate_surface_target(surface_target: ReferenceSurfaceTarget) -> None:
    if surface_target is None:
        raise ValueError("No surface target is available")
    surface_target.surface_point_object.validate()
    surface_target.outward_direction_object.validate()
    surface_target.target_pose_object.validate()
    surface_target.aligned_preapproach_pose_object.validate()
    _validated_distance_delta(
        surface_target.target_surface_distance_m,
        surface_target.aligned_preapproach_distance_m,
    )
