"""Read the hand-to-probe transform from a mount creator archive."""

import hashlib
import json
import math
from pathlib import Path
import stat
import struct
from typing import Union
import zipfile
import zlib

from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_EXPANDED_BYTES = 128 * 1024 * 1024
MAX_ENTRIES = 256
MAX_MANIFEST_BYTES = 64 * 1024
MAX_MESH_TRIANGLES = 200_000


def _read_manifest(archive: zipfile.ZipFile) -> dict:
    entries = archive.infolist()
    names = [entry.filename for entry in entries]
    if len(entries) > MAX_ENTRIES or len(set(names)) != len(names):
        raise ValueError("Archive has too many entries or duplicate filenames")
    if sum(entry.file_size for entry in entries) > MAX_EXPANDED_BYTES:
        raise ValueError("Expanded archive exceeds the 128 MiB limit")
    if any(entry.file_size > MAX_FILE_BYTES for entry in entries):
        raise ValueError("An archive entry exceeds the 32 MiB limit")
    if "manifest.json" not in names:
        raise ValueError("Archive is missing manifest.json")
    if archive.getinfo("manifest.json").file_size > MAX_MANIFEST_BYTES:
        raise ValueError("Manifest exceeds the 64 KiB limit")
    with archive.open("manifest.json") as stream:
        data = stream.read(MAX_MANIFEST_BYTES + 1)
    if len(data) > MAX_MANIFEST_BYTES:
        raise ValueError("Manifest exceeds the 64 KiB limit")
    manifest = json.loads(data)
    if not isinstance(manifest, dict):
        raise ValueError("Manifest must be a JSON object")
    return manifest


def _coordinates(transform: dict, name: str, count: int) -> list:
    values = transform.get(name)
    if not isinstance(values, list) or len(values) != count:
        raise ValueError(f"{name} must contain {count} numbers")
    if not all(
        type(value) in (int, float) and math.isfinite(value)
        for value in values
    ):
        raise ValueError(f"{name} must contain only finite numbers")
    return [float(value) for value in values]


def load_sensor_head_transform(path: Union[str, Path]) -> PoseData:
    """Read a validated transform in metres and quaternion XYZW order.

    Geometry and authoring data are not imported or extracted. File access
    failures raise OSError; invalid archive contents raise ValueError.
    """
    if Path(path).stat().st_size > MAX_FILE_BYTES:
        raise ValueError("Archive exceeds the 32 MiB limit")
    try:
        with zipfile.ZipFile(path) as archive:
            manifest = _read_manifest(archive)
        if type(manifest.get("schema_version")) is not int or (
            manifest["schema_version"] != 1
        ):
            raise ValueError("Unsupported sensor-head schema version")
        if manifest.get("reference_frame") != "hand":
            raise ValueError("Sensor-head reference frame must be 'hand'")
        if manifest.get("units") != "m":
            raise ValueError("Sensor-head units must be metres ('m')")
        transform = manifest.get("hand_to_probe")
        if not isinstance(transform, dict):
            raise ValueError("Manifest must contain a hand_to_probe transform")
        pose = PoseData(
            position=Vector3Data(*_coordinates(transform, "translation", 3)),
            orientation=QuaternionData(
                *_coordinates(transform, "quaternion", 4)
            ),
        )
        pose.validate()
        return pose
    except (
        zipfile.BadZipFile,
        UnicodeError,
        OverflowError,
        RecursionError,
        RuntimeError,
        NotImplementedError,
        zlib.error,
    ) as error:
        raise ValueError(f"Invalid sensor-head archive: {error}") from error


def _has_collision_mesh(archive: zipfile.ZipFile, manifest: dict) -> bool:
    asset = manifest.get("collision_mesh")
    if not isinstance(asset, dict):
        return False
    name = asset.get("path")
    size = asset.get("byte_count")
    checksum = asset.get("sha256")
    if (
        not isinstance(name, str)
        or "\\" in name or ":" in name
        or any(part in ("", ".", "..") for part in name.split("/"))
        or type(size) is not int or not 0 < size <= MAX_FILE_BYTES
        or not isinstance(checksum, str)
    ):
        return False
    entry = archive.getinfo(name)
    file_type = stat.S_IFMT(entry.external_attr >> 16)
    if (
        entry.is_dir() or file_type not in (0, stat.S_IFREG)
        or entry.flag_bits & 1 or entry.file_size != size
    ):
        return False
    with archive.open(entry) as stream:
        data = stream.read(MAX_FILE_BYTES + 1)
    if len(data) != size or hashlib.sha256(data).hexdigest() != checksum:
        return False
    if len(data) < 84:
        return False
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if (
        not 1 <= triangle_count <= MAX_MESH_TRIANGLES
        or len(data) != 84 + 50 * triangle_count
    ):
        return False
    return all(
        math.isfinite(value)
        for triangle in struct.iter_unpack("<12fH", memoryview(data)[84:])
        for value in triangle[:12]
    )


def has_sensor_head_collision_geometry(path: Union[str, Path]) -> bool:
    """Check for a placed binary STL without importing or saving geometry.

    This verifies the asset's integrity and basic structure, not whether its
    shape is suitable for collision planning. Missing or invalid data is false.
    """
    try:
        if Path(path).stat().st_size > MAX_FILE_BYTES:
            return False
        with zipfile.ZipFile(path) as archive:
            manifest = _read_manifest(archive)
            if (
                type(manifest.get("schema_version")) is not int
                or manifest["schema_version"] != 1
                or manifest.get("reference_frame") != "hand"
                or manifest.get("units") != "m"
            ):
                return False
            placement = manifest.get("hand_to_geometry")
            if not isinstance(placement, dict):
                return False
            PoseData(
                position=Vector3Data(*_coordinates(placement, "translation", 3)),
                orientation=QuaternionData(*_coordinates(placement, "quaternion", 4)),
            ).validate()
            return _has_collision_mesh(archive, manifest)
    except (
        OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile,
        OverflowError, RecursionError, RuntimeError, NotImplementedError,
        zlib.error,
    ):
        return False
