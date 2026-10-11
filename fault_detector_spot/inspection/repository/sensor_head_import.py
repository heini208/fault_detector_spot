"""Read sensor-tip transforms and complete mount creator archives."""

from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
import stat
from typing import Union
import zipfile
import zlib

from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_COLLISION_BYTES,
    MAX_MESH_TRIANGLES,
    SensorHeadGeometry,
    validate_collision_mesh,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)


MAX_FILE_BYTES = MAX_COLLISION_BYTES
MAX_EXPANDED_BYTES = 128 * 1024 * 1024
MAX_ENTRIES = 256
MAX_MANIFEST_BYTES = 64 * 1024


@dataclass(frozen=True)
class SensorHeadImport:
    """Validated import data; no files have been extracted or saved yet."""

    hand_to_probe: PoseData
    head_geometry: SensorHeadGeometry
    collision_mesh: bytes


def _validate_asset_path(name: str) -> None:
    if (
        not isinstance(name, str)
        or "\\" in name or ":" in name
        or any(part in ("", ".", "..") for part in name.split("/"))
    ):
        raise ValueError("Archive asset must have a safe relative path")


def _read_manifest(archive: zipfile.ZipFile) -> dict:
    entries = archive.infolist()
    names = [entry.filename for entry in entries]
    if len(entries) > MAX_ENTRIES or len(set(names)) != len(names):
        raise ValueError("Archive has too many entries or duplicate filenames")
    if sum(entry.file_size for entry in entries) > MAX_EXPANDED_BYTES:
        raise ValueError("Expanded archive exceeds the 128 MiB limit")
    for entry in entries:
        _validate_asset_path(entry.filename.rstrip("/") if entry.is_dir() else entry.filename)
        if entry.file_size > MAX_FILE_BYTES:
            raise ValueError("An archive entry exceeds the 32 MiB limit")
        file_type = stat.S_IFMT(entry.external_attr >> 16)
        if file_type not in (0, stat.S_IFREG, stat.S_IFDIR) or entry.flag_bits & 1:
            raise ValueError("Archive entries must be ordinary unencrypted files")
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


@contextmanager
def _open_sensor_head(path: Union[str, Path]):
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
            yield archive, manifest
    except (
        zipfile.BadZipFile,
        UnicodeError,
        OverflowError,
        RecursionError,
        RuntimeError,
        NotImplementedError,
        KeyError,
        zlib.error,
    ) as error:
        raise ValueError(f"Invalid sensor-head archive: {error}") from error


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


def _read_pose(manifest: dict, name: str) -> PoseData:
    transform = manifest.get(name)
    if not isinstance(transform, dict):
        raise ValueError(f"Manifest must contain a {name} transform")
    pose = PoseData(
        position=Vector3Data(*_coordinates(transform, "translation", 3)),
        orientation=QuaternionData(*_coordinates(transform, "quaternion", 4)),
    )
    pose.validate()
    return pose


def load_sensor_head_transform(path: Union[str, Path]) -> PoseData:
    """Read only the validated transform in metres and quaternion XYZW order.

    File access failures raise OSError; invalid archive contents raise ValueError.
    """
    with _open_sensor_head(path) as (_, manifest):
        return _read_pose(manifest, "hand_to_probe")


def _read_collision_mesh(archive: zipfile.ZipFile, manifest: dict) -> bytes:
    asset = manifest.get("collision_mesh")
    if not isinstance(asset, dict):
        raise ValueError("Manifest must describe a collision mesh")
    name = asset.get("path")
    _validate_asset_path(name)
    size = asset.get("byte_count")
    checksum = asset.get("sha256")
    if type(size) is not int or not 84 <= size <= MAX_FILE_BYTES:
        raise ValueError("Collision mesh byte count is invalid")
    if not isinstance(checksum, str) or len(checksum) != 64:
        raise ValueError("Collision mesh SHA-256 checksum is invalid")
    entry = archive.getinfo(name)
    if entry.is_dir() or entry.file_size != size:
        raise ValueError("Collision mesh byte count does not match the archive")
    with archive.open(entry) as stream:
        data = stream.read(MAX_FILE_BYTES + 1)
    validate_collision_mesh(data, checksum, size)
    return data


def load_sensor_head(path: Union[str, Path]) -> SensorHeadImport:
    """Validate the complete package before returning geometry and tip together."""
    with _open_sensor_head(path) as (archive, manifest):
        tip = _read_pose(manifest, "hand_to_probe")
        placement = _read_pose(manifest, "hand_to_geometry")
        asset = manifest.get("collision_mesh")
        if not isinstance(asset, dict):
            raise ValueError("Manifest must describe a collision mesh")
        geometry = SensorHeadGeometry(
            collision_sha256=asset.get("sha256"),
            collision_byte_count=asset.get("byte_count"),
            hand_to_geometry=placement,
            mounting_template_id=manifest.get("mounting_template_id"),
            mounting_template_revision=manifest.get("mounting_template_revision"),
            reference_model_fingerprint=manifest.get("reference_model_fingerprint"),
            mounting_template_verified=manifest.get("mounting_template_verified"),
            clearance_m=manifest.get("clearance_m", 0.0),
            configuration_method=manifest.get("configuration_method", "manual"),
        )
        geometry.validate()
        mesh = _read_collision_mesh(archive, manifest)
        return SensorHeadImport(tip, geometry, mesh)


def has_sensor_head_collision_geometry(path: Union[str, Path]) -> bool:
    """Check for a valid placed mesh without importing or saving any files."""
    try:
        with _open_sensor_head(path) as (archive, manifest):
            _read_pose(manifest, "hand_to_geometry")
            _read_collision_mesh(archive, manifest)
        return True
    except (OSError, ValueError, TypeError):
        return False
