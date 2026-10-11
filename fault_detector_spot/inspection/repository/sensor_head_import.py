"""Read the hand-to-probe transform from a mount creator archive."""

import json
import math
from pathlib import Path
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
