"""Offline tests for bounded uploads and managed collision assets."""

import hashlib
import struct

import pytest

from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_COLLISION_BYTES,
    MAX_MESH_CHUNK_BYTES,
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
    SensorHeadGeometry,
)
from fault_detector_spot.inspection.repository.sensor_mesh_store import SensorMeshStore
from fault_detector_spot.shared.geometry.models import PoseData


@pytest.fixture
def mesh():
    vertices = ((0, 0, 0), (0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    data = bytes(80) + struct.pack("<I", len(faces))
    for face in faces:
        points = [coordinate for index in face for coordinate in vertices[index]]
        data += struct.pack("<12fH", 0, 0, 0, *points, 0)
    return data


def geometry_for(mesh):
    return SensorHeadGeometry(
        hashlib.sha256(mesh).hexdigest(), len(mesh), PoseData.identity(),
        MOUNTING_TEMPLATE_ID, MOUNTING_TEMPLATE_REVISION,
        REFERENCE_MODEL_FINGERPRINT, False,
    )


def test_upload_publishes_only_after_last_chunk_and_survives_restart(tmp_path, mesh):
    store = SensorMeshStore(tmp_path / "assets")
    geometry = geometry_for(mesh)
    digest = geometry.collision_sha256

    assert not store.store_chunk(digest, len(mesh), 0, mesh[:100])
    assert not (store.root_dir / f"{digest}.stl").exists()
    restarted = SensorMeshStore(store.root_dir)
    assert restarted.store_chunk(digest, len(mesh), 100, mesh[100:])

    assert restarted.read(geometry) == mesh
    assert sorted(path.name for path in store.root_dir.iterdir()) == [f"{digest}.stl"]


def test_existing_asset_is_verified_and_reused_without_uploading_remaining_bytes(
    tmp_path, mesh,
):
    store = SensorMeshStore(tmp_path)
    geometry = geometry_for(mesh)
    digest = geometry.collision_sha256
    assert store.store_chunk(digest, len(mesh), 0, mesh)

    assert store.store_chunk(digest, len(mesh), 0, mesh[:20])
    assert store.read(geometry) == mesh
    assert len(list(tmp_path.iterdir())) == 1


def test_wrong_offset_preserves_partial_and_correct_retry_completes(tmp_path, mesh):
    store = SensorMeshStore(tmp_path)
    digest = geometry_for(mesh).collision_sha256
    store.store_chunk(digest, len(mesh), 0, mesh[:100])

    with pytest.raises(ValueError, match="offset"):
        store.store_chunk(digest, len(mesh), 90, mesh[100:])

    assert store.store_chunk(digest, len(mesh), 100, mesh[100:])


def test_offset_zero_restarts_interrupted_upload(tmp_path, mesh):
    store = SensorMeshStore(tmp_path)
    digest = geometry_for(mesh).collision_sha256
    store.store_chunk(digest, len(mesh), 0, b"x" * 100)

    assert store.store_chunk(digest, len(mesh), 0, mesh)
    assert store.read(geometry_for(mesh)) == mesh


@pytest.mark.parametrize("failure", ("checksum", "stl"))
def test_invalid_final_asset_removes_partial_and_allows_retry(tmp_path, mesh, failure):
    store = SensorMeshStore(tmp_path)
    data = b"x" * len(mesh) if failure == "stl" else mesh
    digest = hashlib.sha256(data).hexdigest() if failure == "stl" else "0" * 64
    store.store_chunk(digest, len(data), 0, data[:100])

    with pytest.raises(ValueError):
        store.store_chunk(digest, len(data), 100, data[100:])

    assert list(tmp_path.iterdir()) == []
    geometry = geometry_for(mesh)
    assert store.store_chunk(geometry.collision_sha256, len(mesh), 0, mesh)


@pytest.mark.parametrize(
    "digest,byte_count,offset,data",
    [
        ("../unsafe", 100, 0, b"x"),
        ("A" * 64, 100, 0, b"x"),
        (None, 100, 0, b"x"),
        ("0" * 64, True, 0, b"x"),
        ("0" * 64, 83, 0, b"x"),
        ("0" * 64, MAX_COLLISION_BYTES + 1, 0, b"x"),
        ("0" * 64, 100, -1, b"x"),
        ("0" * 64, 100, True, b"x"),
        ("0" * 64, 100, 0, "x"),
        ("0" * 64, 100, 0, b""),
        ("0" * 64, 100, 100, b"x"),
        ("0" * 64, MAX_COLLISION_BYTES, 0, bytes(MAX_MESH_CHUNK_BYTES + 1)),
    ],
)
def test_invalid_chunk_does_not_create_storage(
    tmp_path, digest, byte_count, offset, data,
):
    store = SensorMeshStore(tmp_path / "assets")

    with pytest.raises(ValueError):
        store.store_chunk(digest, byte_count, offset, data)

    assert not store.root_dir.exists()


@pytest.mark.parametrize("target", ("directory", "asset", "partial"))
def test_rejects_symlink_paths_without_changing_target(tmp_path, mesh, target):
    asset_dir = tmp_path / "assets"
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    protected = elsewhere / "protected"
    protected.write_bytes(b"keep")
    geometry = geometry_for(mesh)
    digest = geometry.collision_sha256
    if target == "directory":
        asset_dir.symlink_to(elsewhere, target_is_directory=True)
    else:
        asset_dir.mkdir()
        suffix = "stl" if target == "asset" else "part"
        (asset_dir / f"{digest}.{suffix}").symlink_to(protected)

    with pytest.raises(ValueError):
        SensorMeshStore(asset_dir).store_chunk(digest, len(mesh), 0, mesh)

    assert protected.read_bytes() == b"keep"


@pytest.mark.parametrize("content", (b"bad", None))
def test_missing_or_tampered_asset_is_not_returned(tmp_path, mesh, content):
    store = SensorMeshStore(tmp_path)
    geometry = geometry_for(mesh)
    if content is not None:
        (tmp_path / f"{geometry.collision_sha256}.stl").write_bytes(content)

    with pytest.raises((ValueError, FileNotFoundError)):
        store.read(geometry)


def test_corrupt_complete_asset_is_not_silently_replaced(tmp_path, mesh):
    store = SensorMeshStore(tmp_path)
    geometry = geometry_for(mesh)
    path = tmp_path / f"{geometry.collision_sha256}.stl"
    path.write_bytes(b"corrupt")

    with pytest.raises(ValueError):
        store.store_chunk(geometry.collision_sha256, len(mesh), 0, mesh)

    assert path.read_bytes() == b"corrupt"


def test_equal_length_asset_tampering_fails_checksum(tmp_path, mesh):
    store = SensorMeshStore(tmp_path)
    geometry = geometry_for(mesh)
    path = tmp_path / f"{geometry.collision_sha256}.stl"
    path.write_bytes(b"x" + mesh[1:])

    with pytest.raises(ValueError, match="checksum"):
        store.read(geometry)


def test_failed_publish_discards_partial_without_creating_asset(
    tmp_path, mesh, monkeypatch,
):
    store = SensorMeshStore(tmp_path)
    digest = geometry_for(mesh).collision_sha256

    def fail_replace(*_args):
        raise OSError("simulated publish failure")

    monkeypatch.setattr(
        "fault_detector_spot.inspection.repository.sensor_mesh_store.os.replace",
        fail_replace,
    )
    with pytest.raises(OSError, match="publish failure"):
        store.store_chunk(digest, len(mesh), 0, mesh)

    assert list(tmp_path.iterdir()) == []
