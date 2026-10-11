"""Archive boundary tests for importing a sensor-tip transform."""

import hashlib
import json
import math
import stat
import struct
import zipfile

import pytest

from fault_detector_spot.inspection.repository import sensor_head_import
from fault_detector_spot.inspection.repository.sensor_head_import import (
    has_sensor_head_collision_geometry,
    load_sensor_head,
    load_sensor_head_transform,
)
from fault_detector_spot.inspection.model.sensor_geometry import (
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
)


@pytest.fixture
def manifest():
    return {
        "schema_version": 1,
        "reference_frame": "hand",
        "units": "m",
        "mounting_template_id": MOUNTING_TEMPLATE_ID,
        "mounting_template_revision": MOUNTING_TEMPLATE_REVISION,
        "reference_model_fingerprint": REFERENCE_MODEL_FINGERPRINT,
        "mounting_template_verified": False,
        "clearance_m": 0.0,
        "configuration_method": "manual",
        "hand_to_probe": {
            "translation": [0.046238241757377466, -0.00107, -0.079173945],
            "quaternion": [0.0, math.sin(0.2), 0.0, math.cos(0.2)],
        },
    }


def write_archive(tmp_path, manifest, extra_entries=()):
    path = tmp_path / "head.sensorhead"
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
        for name, data in extra_entries:
            archive.writestr(name, data)
    return path


def test_import_preserves_metres_and_xyzw_without_reading_geometry(
    tmp_path, manifest
):
    manifest["hand_to_geometry"] = {
        "translation": [1, 2, 3], "quaternion": [0, 0, 0, 1]
    }
    path = write_archive(
        tmp_path, manifest, [("collision.stl", b"not a mesh")]
    )

    pose = load_sensor_head_transform(path)

    assert list(pose.position.to_dict().values()) == (
        manifest["hand_to_probe"]["translation"]
    )
    assert list(pose.orientation.to_dict().values()) == (
        manifest["hand_to_probe"]["quaternion"]
    )
    assert sorted(item.name for item in tmp_path.iterdir()) == [
        "head.sensorhead"
    ]


@pytest.mark.parametrize(
    "field,value",
    [
        ("schema_version", True),
        ("schema_version", 1.0),
        ("schema_version", 2),
        ("reference_frame", "body"),
        ("units", "mm"),
        ("hand_to_probe", None),
    ],
)
def test_rejects_incompatible_manifest(tmp_path, manifest, field, value):
    manifest[field] = value

    with pytest.raises(ValueError):
        load_sensor_head_transform(write_archive(tmp_path, manifest))


@pytest.mark.parametrize(
    "field,value",
    [
        ("translation", [1, 2]),
        ("translation", [1, 2, "3"]),
        ("translation", [True, 2, 3]),
        ("translation", [float("nan"), 0, 0]),
        ("translation", [float("inf"), 0, 0]),
        ("quaternion", [0, 0, 0]),
        ("quaternion", [0, 0, 0, 0]),
        ("quaternion", [0, 0, 0, 2]),
        ("quaternion", [0, 0, 0, True]),
        ("quaternion", [1e308, 0, 0, 1]),
    ],
)
def test_rejects_invalid_transform(tmp_path, manifest, field, value):
    manifest["hand_to_probe"][field] = value

    with pytest.raises(ValueError):
        load_sensor_head_transform(write_archive(tmp_path, manifest))


@pytest.mark.parametrize("content", [b"{", b"\xff", b"[]"])
def test_rejects_invalid_manifest_json(tmp_path, content):
    path = tmp_path / "bad.zip"
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("manifest.json", content)

    with pytest.raises(ValueError):
        load_sensor_head_transform(path)


def test_rejects_missing_root_manifest(tmp_path):
    path = tmp_path / "nested.zip"
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("folder/manifest.json", "{}")

    with pytest.raises(ValueError, match="missing manifest.json"):
        load_sensor_head_transform(path)


def test_rejects_duplicate_manifest(tmp_path, manifest):
    with pytest.warns(UserWarning, match="Duplicate name"):
        path = write_archive(tmp_path, manifest, [("manifest.json", "{}")])

    with pytest.raises(ValueError, match="duplicate"):
        load_sensor_head_transform(path)


def test_rejects_oversized_compressed_manifest(tmp_path):
    path = tmp_path / "large.zip"
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(
            "manifest.json", b" " * (sensor_head_import.MAX_MANIFEST_BYTES + 1)
        )

    with pytest.raises(ValueError, match="Manifest exceeds"):
        load_sensor_head_transform(path)


@pytest.mark.parametrize(
    "limit,value,message",
    [
        ("MAX_FILE_BYTES", 1, "Archive exceeds"),
        ("MAX_EXPANDED_BYTES", 1, "Expanded archive exceeds"),
        ("MAX_ENTRIES", 0, "too many entries"),
    ],
)
def test_enforces_archive_limits(
    tmp_path, manifest, monkeypatch, limit, value, message
):
    path = write_archive(tmp_path, manifest)
    monkeypatch.setattr(sensor_head_import, limit, value)

    with pytest.raises(ValueError, match=message):
        load_sensor_head_transform(path)


def test_rejects_non_zip_file(tmp_path):
    path = tmp_path / "bad.zip"
    path.write_text("not an archive")

    with pytest.raises(ValueError, match="Invalid sensor-head archive"):
        load_sensor_head_transform(path)


def test_file_access_failure_is_preserved(tmp_path):
    with pytest.raises(OSError):
        load_sensor_head_transform(tmp_path / "missing.sensorhead")


@pytest.fixture
def collision_mesh():
    vertices = ((0, 0, 0), (.1, 0, 0), (0, .1, 0), (0, 0, .1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    return b"\0" * 80 + struct.pack("<I", len(faces)) + b"".join(
        struct.pack(
            "<12fH", 0, 0, 0,
            *(coordinate for index in face for coordinate in vertices[index]), 0,
        )
        for face in faces
    )


def add_collision_description(manifest, data):
    manifest["hand_to_geometry"] = {
        "translation": [0, 0, 0], "quaternion": [0, 0, 0, 1],
    }
    manifest["collision_mesh"] = {
        "path": "collision.stl",
        "byte_count": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def test_finds_collision_geometry_without_extracting_files(
    tmp_path, manifest, collision_mesh,
):
    add_collision_description(manifest, collision_mesh)
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])

    assert has_sensor_head_collision_geometry(path)
    assert sorted(item.name for item in tmp_path.iterdir()) == ["head.sensorhead"]


@pytest.mark.parametrize("missing", ("description", "member", "placement"))
def test_missing_collision_data_is_false(tmp_path, manifest, collision_mesh, missing):
    add_collision_description(manifest, collision_mesh)
    entries = [("collision.stl", collision_mesh)]
    if missing == "description":
        del manifest["collision_mesh"]
    elif missing == "placement":
        del manifest["hand_to_geometry"]
    else:
        entries = []
    path = write_archive(tmp_path, manifest, entries)

    assert not has_sensor_head_collision_geometry(path)
    assert load_sensor_head_transform(path).position.x == manifest["hand_to_probe"]["translation"][0]


@pytest.mark.parametrize("field,value", (
    ("path", "../collision.stl"), ("path", "/collision.stl"),
    ("path", "folder\\collision.stl"), ("path", "C:collision.stl"),
    ("path", None), ("byte_count", True), ("byte_count", 0),
    ("byte_count", 99), ("sha256", "0" * 64), ("sha256", None),
))
def test_rejects_invalid_collision_description(
    tmp_path, manifest, collision_mesh, field, value,
):
    add_collision_description(manifest, collision_mesh)
    manifest["collision_mesh"][field] = value
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])

    assert not has_sensor_head_collision_geometry(path)


@pytest.mark.parametrize("damage", (
    "truncated", "empty", "nan_vertex", "inf_normal", "triangle_limit",
))
def test_rejects_invalid_stl_even_with_matching_checksum(
    tmp_path, manifest, collision_mesh, damage,
):
    data = bytearray(collision_mesh)
    if damage == "truncated":
        data.pop()
    elif damage == "empty":
        data = bytearray(84)
    elif damage == "nan_vertex":
        struct.pack_into("<f", data, 96, math.nan)
    elif damage == "inf_normal":
        struct.pack_into("<f", data, 84, math.inf)
    else:
        struct.pack_into("<I", data, 80, sensor_head_import.MAX_MESH_TRIANGLES + 1)
    add_collision_description(manifest, data)
    path = write_archive(tmp_path, manifest, [("collision.stl", data)])

    assert not has_sensor_head_collision_geometry(path)
    load_sensor_head_transform(path)


@pytest.mark.parametrize("field,value", (
    ("translation", [0, math.nan, 0]),
    ("quaternion", [0, 0, 0, 0]),
))
def test_rejects_invalid_collision_placement(
    tmp_path, manifest, collision_mesh, field, value,
):
    add_collision_description(manifest, collision_mesh)
    manifest["hand_to_geometry"][field] = value
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])

    assert not has_sensor_head_collision_geometry(path)
    load_sensor_head_transform(path)


def test_rejects_collision_symlink(tmp_path, manifest, collision_mesh):
    add_collision_description(manifest, collision_mesh)
    entry = zipfile.ZipInfo("collision.stl")
    entry.create_system = 3
    entry.external_attr = (stat.S_IFLNK | 0o777) << 16
    path = write_archive(tmp_path, manifest, [(entry, collision_mesh)])

    assert not has_sensor_head_collision_geometry(path)


@pytest.mark.parametrize("limit", (
    "MAX_FILE_BYTES", "MAX_EXPANDED_BYTES", "MAX_ENTRIES", "MAX_MANIFEST_BYTES",
))
def test_collision_status_obeys_archive_limits(
    tmp_path, manifest, collision_mesh, monkeypatch, limit,
):
    add_collision_description(manifest, collision_mesh)
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])
    monkeypatch.setattr(sensor_head_import, limit, 1)

    assert not has_sensor_head_collision_geometry(path)


def test_collision_status_handles_unreadable_archive(tmp_path):
    path = tmp_path / "head.sensorhead"
    assert not has_sensor_head_collision_geometry(path)
    path.write_bytes(b"Not a ZIP archive")
    assert not has_sensor_head_collision_geometry(path)


def test_full_import_preserves_pose_mesh_and_unverified_provenance(
    tmp_path, manifest, collision_mesh,
):
    add_collision_description(manifest, collision_mesh)
    manifest["hand_to_geometry"]["translation"] = [-.112, .042, -.065]
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])

    imported = load_sensor_head(path)

    assert imported.collision_mesh == collision_mesh
    assert imported.hand_to_probe == load_sensor_head_transform(path)
    assert imported.head_geometry.hand_to_geometry.position.x == -.112
    assert imported.head_geometry.mounting_template_verified is False
    assert imported.head_geometry.asset_path == (
        f"assets/{hashlib.sha256(collision_mesh).hexdigest()}.stl"
    )
    assert sorted(item.name for item in tmp_path.iterdir()) == ["head.sensorhead"]


@pytest.mark.parametrize("field,value", (
    ("mounting_template_id", "different-template"),
    ("mounting_template_revision", True),
    ("mounting_template_revision", 2),
    ("reference_model_fingerprint", "0" * 64),
    ("mounting_template_verified", "false"),
    ("clearance_m", .001),
    ("configuration_method", "unknown"),
))
def test_full_import_rejects_unsupported_metadata(
    tmp_path, manifest, collision_mesh, field, value,
):
    add_collision_description(manifest, collision_mesh)
    manifest[field] = value
    path = write_archive(tmp_path, manifest, [("collision.stl", collision_mesh)])

    with pytest.raises(ValueError):
        load_sensor_head(path)


@pytest.mark.parametrize("damage", ("missing", "checksum", "open", "placement"))
def test_full_import_rejects_invalid_geometry_as_one_package(
    tmp_path, manifest, collision_mesh, damage,
):
    if damage == "open":
        collision_mesh = (
            collision_mesh[:80] + struct.pack("<I", 3) + collision_mesh[84:-50]
        )
    add_collision_description(manifest, collision_mesh)
    entries = [("collision.stl", collision_mesh)]
    if damage == "missing":
        entries = []
    elif damage == "checksum":
        manifest["collision_mesh"]["sha256"] = "0" * 64
    elif damage == "placement":
        manifest["hand_to_geometry"]["quaternion"] = [0, 0, 0, 0]
    path = write_archive(tmp_path, manifest, entries)

    with pytest.raises(ValueError):
        load_sensor_head(path)
    assert load_sensor_head_transform(path).position.x == (
        manifest["hand_to_probe"]["translation"][0]
    )


@pytest.mark.parametrize("name", ("../payload", "/payload", "folder\\payload"))
def test_full_import_rejects_unsafe_unused_entries(
    tmp_path, manifest, collision_mesh, name,
):
    add_collision_description(manifest, collision_mesh)
    path = write_archive(
        tmp_path, manifest, [("collision.stl", collision_mesh), (name, b"unused")]
    )

    with pytest.raises(ValueError, match="safe relative path"):
        load_sensor_head(path)
