"""Resolve explicit probe alignment orientations for setup workflows."""

from copy import deepcopy
import math

import numpy as np

from fault_detector_spot.inspection.geometry.rotation import (
    inverse_quaternion,
    multiply_quaternions,
    quaternion_from_matrix,
    rotate_vector,
)
from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)


_TAG_FACING_PITCH_SIN = math.sin(math.pi / 4.0)
_TAG_FACING_PITCH_COS = math.cos(math.pi / 4.0)


def tag_aligned_probe_orientation(
    hand_to_probe_orientation: QuaternionData,
) -> QuaternionData:
    """Return the probe orientation produced by relative-to-tag mode."""
    hand_to_probe_orientation.validate()
    return QuaternionData(
        x=0.0,
        y=_TAG_FACING_PITCH_SIN,
        z=0.0,
        w=_TAG_FACING_PITCH_COS,
    )


def surface_aligned_probe_orientation(
    outward_direction_object: Vector3Data,
    hand_to_probe_orientation: QuaternionData,
    gravity_up_object: Vector3Data,
) -> QuaternionData:
    """Face the surface while using gravity only to resolve axial roll."""
    outward_direction_object.validate()
    hand_to_probe_orientation.validate()
    gravity_up_object.validate()

    local_x = -_normalized(_array(outward_direction_object))
    preferred_up = _normalized(_array(gravity_up_object))
    local_z = preferred_up - np.dot(preferred_up, local_x) * local_x
    if float(np.linalg.norm(local_z)) <= 1e-8:
        fallback = np.array([0.0, 1.0, 0.0], dtype=float)
        if abs(float(np.dot(fallback, local_x))) > 0.95:
            fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        local_z = fallback - np.dot(fallback, local_x) * local_x
    local_z = _normalized(local_z)
    local_y = _normalized(np.cross(local_z, local_x))
    local_z = _normalized(np.cross(local_x, local_y))
    base = quaternion_from_matrix(
        np.column_stack((local_x, local_y, local_z))
    )
    return _upright_roll_only(
        base,
        hand_to_probe_orientation,
        gravity_up_object,
    )


def _upright_roll_only(
    probe_orientation: QuaternionData,
    hand_to_probe_orientation: QuaternionData,
    gravity_up_object: Vector3Data,
) -> QuaternionData:
    gravity_up_probe = rotate_vector(
        inverse_quaternion(probe_orientation),
        gravity_up_object,
    )
    hand_up_probe = rotate_vector(
        inverse_quaternion(hand_to_probe_orientation),
        Vector3Data(x=0.0, y=0.0, z=1.0),
    )
    cosine_term = (
        gravity_up_probe.y * hand_up_probe.y
        + gravity_up_probe.z * hand_up_probe.z
    )
    sine_term = (
        -gravity_up_probe.y * hand_up_probe.z
        + gravity_up_probe.z * hand_up_probe.y
    )
    if math.hypot(cosine_term, sine_term) <= 1e-12:
        return deepcopy(probe_orientation)
    roll_rad = math.atan2(sine_term, cosine_term)
    half_roll = roll_rad * 0.5
    return multiply_quaternions(
        probe_orientation,
        QuaternionData(
            x=math.sin(half_roll),
            y=0.0,
            z=0.0,
            w=math.cos(half_roll),
        ),
    )


def _array(vector: Vector3Data) -> np.ndarray:
    return np.array([vector.x, vector.y, vector.z], dtype=float)


def _normalized(values) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("Alignment direction must contain three finite values")
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Alignment direction cannot be normalized")
    return values / norm


__all__ = [
    "surface_aligned_probe_orientation",
    "tag_aligned_probe_orientation",
]
