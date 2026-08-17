"""Shared 3D rotation helpers backed by SciPy."""

import math
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)


def rotation_from_quaternion(quaternion: QuaternionData) -> Rotation:
    quaternion.validate()
    values = np.array(
        [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("Quaternion contains non-finite values")
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Quaternion has zero norm")
    return Rotation.from_quat(values / norm)


def quaternion_from_rotation(rotation: Rotation) -> QuaternionData:
    if not isinstance(rotation, Rotation):
        raise TypeError("Expected a SciPy Rotation")
    values = np.asarray(rotation.as_quat(), dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("SciPy returned an invalid quaternion")
    result = QuaternionData(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
        w=float(values[3]),
    )
    result.validate()
    return result


def quaternion_from_matrix(matrix) -> QuaternionData:
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Rotation matrix is not finite")
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6):
        raise ValueError("Rotation matrix is not orthonormal")
    if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-6):
        raise ValueError("Rotation matrix is not right-handed")
    return quaternion_from_rotation(Rotation.from_matrix(matrix))


def rotate_vector(
    quaternion: QuaternionData,
    vector: Vector3Data,
) -> Vector3Data:
    quaternion.validate()
    vector.validate()
    rotated = rotation_from_quaternion(quaternion).apply(
        np.array([vector.x, vector.y, vector.z], dtype=float)
    )
    return Vector3Data(
        x=float(rotated[0]),
        y=float(rotated[1]),
        z=float(rotated[2]),
    )


def quaternion_to_rpy(
    quaternion: QuaternionData,
) -> tuple[float, float, float]:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        values = rotation_from_quaternion(quaternion).as_euler(
            "xyz",
            degrees=False,
        )
    return float(values[0]), float(values[1]), float(values[2])


__all__ = [
    "quaternion_from_matrix",
    "quaternion_from_rotation",
    "quaternion_to_rpy",
    "rotate_vector",
    "rotation_from_quaternion",
]
