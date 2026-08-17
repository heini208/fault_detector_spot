"""Transient probe-point setup poses and frame conversion helpers."""

import math
from dataclasses import dataclass

import numpy as np

from fault_detector_spot.inspection.geometry.rotation import (
    inverse_quaternion,
    multiply_quaternions as scipy_multiply_quaternions,
    quaternion_from_euler,
    rotate_vector as scipy_rotate_vector,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    validate_surface_distance_pair,
)
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
    hand_to_probe_pose: PoseData = None,
) -> ReferenceProbeSetup:
    """Create transient setup state from calculated surface geometry."""
    _validate_surface_target(surface_target)
    mounting = hand_to_probe_pose or PoseData.identity()
    mounting.validate()
    probe_orientation = _level_hand_probe_orientation(
        surface_target.target_pose_object.orientation,
        mounting.orientation,
    )
    aligned_pose = PoseData(
        position=surface_target.aligned_preapproach_pose_object.position,
        orientation=probe_orientation,
    )
    probe_pose = PoseData(
        position=surface_target.target_pose_object.position,
        orientation=probe_orientation,
    )
    return ReferenceProbeSetup(
        surface_target=surface_target,
        safe_approach_pose_object=aligned_pose,
        aligned_preapproach_pose_object=aligned_pose,
        probe_pose_object=probe_pose,
    )


def _level_hand_probe_orientation(
    desired_surface_orientation: QuaternionData,
    hand_to_probe_orientation: QuaternionData,
) -> QuaternionData:
    desired_surface_orientation.validate()
    hand_to_probe_orientation.validate()
    local_x = Vector3Data(x=1.0, y=0.0, z=0.0)
    desired_axis = rotate_vector(desired_surface_orientation, local_x)
    mounted_axis = rotate_vector(hand_to_probe_orientation, local_x)
    desired = np.array(
        [desired_axis.x, desired_axis.y, desired_axis.z],
        dtype=float,
    )
    mounted = np.array(
        [mounted_axis.x, mounted_axis.y, mounted_axis.z],
        dtype=float,
    )
    pitch_radius = math.hypot(mounted[0], mounted[2])
    if pitch_radius <= 1e-10:
        if abs(desired[2]) > 1e-8:
            raise ValueError(
                "A zero-roll hand pose cannot align this sensor mounting"
            )
        pitch_candidates = (0.0,)
    else:
        ratio = desired[2] / pitch_radius
        if abs(ratio) > 1.0 + 1e-8:
            raise ValueError(
                "A zero-roll hand pose cannot align this sensor mounting"
            )
        ratio = max(-1.0, min(1.0, ratio))
        phase = math.atan2(mounted[0], mounted[2])
        solution = math.acos(ratio)
        pitch_candidates = (
            _normalize_angle(solution - phase),
            _normalize_angle(-solution - phase),
        )

    candidates = []
    desired_values = np.array(
        [
            desired_surface_orientation.x,
            desired_surface_orientation.y,
            desired_surface_orientation.z,
            desired_surface_orientation.w,
        ],
        dtype=float,
    )
    for pitch in pitch_candidates:
        pitched_x = (
            math.cos(pitch) * mounted[0]
            + math.sin(pitch) * mounted[2]
        )
        yaw = _normalize_angle(
            math.atan2(desired[1], desired[0])
            - math.atan2(mounted[1], pitched_x)
        )
        hand_orientation = _pitch_yaw_quaternion(pitch, yaw)
        probe_orientation = multiply_quaternions(
            hand_orientation,
            hand_to_probe_orientation,
        )
        achieved_axis = rotate_vector(probe_orientation, local_x)
        error = float(
            np.linalg.norm(
                np.array(
                    [
                        achieved_axis.x,
                        achieved_axis.y,
                        achieved_axis.z,
                    ]
                )
                - desired
            )
        )
        hand_values = np.array(
            [
                hand_orientation.x,
                hand_orientation.y,
                hand_orientation.z,
                hand_orientation.w,
            ],
            dtype=float,
        )
        orientation_delta = 1.0 - abs(
            float(np.dot(hand_values, desired_values))
        )
        candidates.append(
            (error, orientation_delta, probe_orientation)
        )

    candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
    if not candidates or candidates[0][0] > 1e-7:
        raise ValueError(
            "A zero-roll hand pose cannot align this sensor mounting"
        )
    return candidates[0][2]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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
    inverse_orientation = inverse_quaternion(pose.orientation)
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
    """Compose two rotations with SciPy."""
    return scipy_multiply_quaternions(first, second)


def rotate_vector(
    quaternion: QuaternionData,
    vector: Vector3Data,
) -> Vector3Data:
    """Rotate a vector with the shared SciPy geometry adapter."""
    return scipy_rotate_vector(quaternion, vector)


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
    pitch = quaternion_from_euler("y", pitch_rad)
    yaw = quaternion_from_euler("z", yaw_rad)
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
