"""Archive boundary tests for importing a sensor-tip transform."""

import json
import math
import zipfile

import pytest

from fault_detector_spot.inspection.repository import sensor_head_import
from fault_detector_spot.inspection.repository.sensor_head_import import (
    load_sensor_head_transform,
)


@pytest.fixture
def manifest():
    return {
        "schema_version": 1,
        "reference_frame": "hand",
        "units": "m",
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
