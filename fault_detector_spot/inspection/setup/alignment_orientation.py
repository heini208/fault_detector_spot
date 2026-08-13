"""Resolve explicit probe alignment orientations for setup workflows."""

from copy import deepcopy
import math

import numpy as np

from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    multiply_quaternions,
    rotate_vector,
)


ALIGNMENT_ORIENTATION_TAG = "tag"
ALIGNMENT_ORIENTATION_CALCULATED_SURFACE = "calculated_surface"


def tag_aligned_probe_orientation(
    hand_to_probe_orientation: QuaternionData,
) -> QuaternionData:
    """Return the probe orientation produced by a tag-aligned hand."""
    hand_to_probe_orientation.validate()
    return deepcopy(hand_to_probe_orientation)


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
    base = _matrix_to_quaternion(
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
        _inverse(probe_orientation),
        gravity_up_object,
    )
    hand_up_probe = rotate_vector(
        _inverse(hand_to_probe_orientation),
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


def _inverse(orientation: QuaternionData) -> QuaternionData:
    orientation.validate()
    return QuaternionData(
        x=-orientation.x,
        y=-orientation.y,
        z=-orientation.z,
        w=orientation.w,
    )


def _array(vector: Vector3Data) -> np.ndarray:
    return np.array([vector.x, vector.y, vector.z], dtype=float)


def _normalized(values) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(values))
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("Alignment direction must contain three finite values")
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Alignment direction cannot be normalized")
    return values / norm


def _matrix_to_quaternion(rotation: np.ndarray) -> QuaternionData:
    rotation = np.asarray(rotation, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        ) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
        ) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    values = np.array([x, y, z, w], dtype=float)
    values /= np.linalg.norm(values)
    result = QuaternionData(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
        w=float(values[3]),
    )
    result.validate()
    return result


__all__ = [
    "ALIGNMENT_ORIENTATION_CALCULATED_SURFACE",
    "ALIGNMENT_ORIENTATION_TAG",
    "surface_aligned_probe_orientation",
    "tag_aligned_probe_orientation",
]
