"""Tests for shared SciPy-backed rotation geometry."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_matrix,
    quaternion_to_rpy,
    rotate_vector,
    rotation_from_quaternion,
)
from fault_detector_spot.inspection.model.models import (
    QuaternionData,
    Vector3Data,
)


def _normalized_quaternion(x, y, z, w):
    values = np.array([x, y, z, w], dtype=float)
    values /= np.linalg.norm(values)
    return QuaternionData(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
        w=float(values[3]),
    )


def test_quaternion_round_trip_preserves_rotation():
    source = _normalized_quaternion(
        0.2,
        -0.3,
        0.1,
        0.9,
    )

    rotation = rotation_from_quaternion(source)
    rebuilt = quaternion_from_matrix(rotation.as_matrix())

    expected = rotation.as_matrix()
    actual = rotation_from_quaternion(rebuilt).as_matrix()
    assert actual == pytest.approx(expected)


def test_rotate_vector_matches_scipy():
    quaternion = QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sqrt(0.5),
        w=math.sqrt(0.5),
    )
    vector = Vector3Data(x=1.0, y=0.0, z=0.0)

    result = rotate_vector(quaternion, vector)

    expected = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    ).apply([1.0, 0.0, 0.0])
    assert [result.x, result.y, result.z] == pytest.approx(expected)


def test_quaternion_to_rpy_matches_scipy_xyz():
    quaternion = _normalized_quaternion(
        0.1,
        -0.2,
        0.3,
        0.9,
    )

    result = quaternion_to_rpy(quaternion)

    expected = Rotation.from_quat(
        [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]
    ).as_euler("xyz")
    assert result == pytest.approx(expected)


def test_matrix_conversion_rejects_non_rotation():
    with pytest.raises(ValueError, match="orthonormal"):
        quaternion_from_matrix(
            np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 2.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
        )
