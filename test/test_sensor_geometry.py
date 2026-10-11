"""Integrity checks for persisted sensor-head collision assets."""

from dataclasses import replace
import hashlib
import math
import struct

import pytest

from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_MESH_TRIANGLES,
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
    SensorHeadGeometry,
    validate_collision_mesh,
)
from fault_detector_spot.shared.geometry.models import PoseData


def binary_stl(triangles):
    return b"\0" * 80 + struct.pack("<I", len(triangles)) + b"".join(
        struct.pack(
            "<12fH", 0, 0, 0,
            *(coordinate for point in triangle for coordinate in point), 0,
        )
        for triangle in triangles
    )


@pytest.fixture
def tetrahedron():
    points = ((0, 0, 0), (.1, 0, 0), (0, .1, 0), (0, 0, .1))
    return [
        [points[index] for index in face]
        for face in ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    ]


@pytest.fixture
def geometry(tetrahedron):
    data = binary_stl(tetrahedron)
    return SensorHeadGeometry(
        collision_sha256=hashlib.sha256(data).hexdigest(),
        collision_byte_count=len(data),
        hand_to_geometry=PoseData.identity(),
        mounting_template_id=MOUNTING_TEMPLATE_ID,
        mounting_template_revision=MOUNTING_TEMPLATE_REVISION,
        reference_model_fingerprint=REFERENCE_MODEL_FINGERPRINT,
        mounting_template_verified=False,
    )


def test_geometry_round_trip_preserves_unverified_metadata(geometry):
    restored = SensorHeadGeometry.from_dict(geometry.to_dict())

    assert restored == geometry
    assert restored.asset_path == f"assets/{geometry.collision_sha256}.stl"
    assert "asset_path" not in restored.to_dict()


@pytest.mark.parametrize("field,value", (
    ("collision_sha256", "../untrusted"),
    ("collision_sha256", "A" * 64),
    ("collision_sha256", None),
    ("collision_byte_count", True),
    ("collision_byte_count", 0),
    ("collision_byte_count", 32 * 1024 * 1024 + 1),
    ("mounting_template_revision", True),
    ("mounting_template_verified", 0),
    ("reference_model_fingerprint", "0" * 64),
    ("hand_to_geometry", None),
    ("clearance_m", math.nan),
    ("clearance_m", True),
    ("clearance_m", .001),
))
def test_geometry_rejects_invalid_metadata(geometry, field, value):
    with pytest.raises(ValueError):
        replace(geometry, **{field: value}).validate()


def test_closed_mesh_preserves_bytes_and_checks_identity(tetrahedron):
    data = binary_stl(tetrahedron)
    validate_collision_mesh(data, hashlib.sha256(data).hexdigest(), len(data))

    with pytest.raises(ValueError, match="checksum"):
        validate_collision_mesh(data, "0" * 64)
    with pytest.raises(ValueError, match="byte count"):
        validate_collision_mesh(data, expected_byte_count=len(data) + 1)


@pytest.mark.parametrize("damage", (
    "open", "reversed_face", "degenerate", "flat", "flat_component", "nan",
))
def test_mesh_rejects_unsuitable_geometry(tetrahedron, damage):
    if damage == "open":
        tetrahedron.pop()
    elif damage == "reversed_face":
        tetrahedron[0].reverse()
    elif damage == "degenerate":
        tetrahedron[0][0] = tetrahedron[0][1]
    elif damage in ("flat", "flat_component"):
        flat = [[(x + 1, y, 0) for x, y, z in face] for face in tetrahedron]
        # A planar double-sided triangle passes closed-edge checks but has no volume.
        flat = [flat[-1], list(reversed(flat[-1]))]
        tetrahedron = flat if damage == "flat" else tetrahedron + flat
    else:
        tetrahedron[0][0] = (math.nan, 0, 0)

    with pytest.raises(ValueError):
        validate_collision_mesh(binary_stl(tetrahedron))


def test_distinct_components_may_have_opposite_winding(tetrahedron):
    second = [
        [(x + 1, y, z) for x, y, z in reversed(face)]
        for face in tetrahedron
    ]
    validate_collision_mesh(binary_stl(tetrahedron + second))


@pytest.mark.parametrize("damage", ("truncated", "trailing", "count", "normal"))
def test_mesh_rejects_invalid_binary_stl(tetrahedron, damage):
    data = bytearray(binary_stl(tetrahedron))
    if damage == "truncated":
        data.pop()
    elif damage == "trailing":
        data.append(0)
    elif damage == "count":
        struct.pack_into("<I", data, 80, MAX_MESH_TRIANGLES + 1)
    else:
        struct.pack_into("<f", data, 84, math.inf)

    with pytest.raises(ValueError):
        validate_collision_mesh(bytes(data))
