"""Verify bounded collision downloads over generated ROS service messages."""

import hashlib
import struct
from types import SimpleNamespace

import pytest
from fault_detector_msgs.srv import GetSensorMesh
from rclpy.serialization import deserialize_message, serialize_message

from fault_detector_spot.inspection.model.sensor_geometry import MAX_MESH_CHUNK_BYTES

from test_sensor_mesh_store import mesh
from test_sensor_registry_creation import api_state, definition, queued_command


@pytest.fixture
def stored_asset(tmp_path, mesh):
    api, attachment, commands, repository = api_state(tmp_path)
    # Closed tetrahedra provide a valid mesh large enough for several chunks.
    data = bytes(80) + struct.pack("<I", 2000) + mesh[84:] * 500
    digest = hashlib.sha256(data).hexdigest()
    for offset in range(0, len(data), MAX_MESH_CHUNK_BYTES):
        repository.store_collision_chunk(
            digest, len(data), offset, data[offset:offset + MAX_MESH_CHUNK_BYTES],
        )
    return SimpleNamespace(
        api=api, attachment=attachment, commands=commands,
        repository=repository, data=data, digest=digest,
    )


def download(asset, offset=0, **values):
    request = GetSensorMesh.Request(
        sha256=values.get("sha256", asset.digest),
        byte_count=values.get("byte_count", len(asset.data)),
        offset=offset,
    )
    request = deserialize_message(serialize_message(request), GetSensorMesh.Request)
    response = asset.api._handle_get_mesh(request, GetSensorMesh.Response())
    return deserialize_message(serialize_message(response), GetSensorMesh.Response)


def test_generated_service_download_reconstructs_exact_mesh_in_bounded_chunks(
    stored_asset,
):
    restored = bytearray()
    complete = False
    while not complete:
        response = download(stored_asset, offset=len(restored))
        assert response.success, response.message
        assert 0 < len(response.data) <= MAX_MESH_CHUNK_BYTES
        restored.extend(response.data)
        complete = response.complete
        assert complete == (len(restored) == len(stored_asset.data))

    assert bytes(restored) == stored_asset.data
    assert hashlib.sha256(restored).hexdigest() == stored_asset.digest
    assert "49152" in GetSensorMesh.Response.get_fields_and_field_types()["data"]


def test_download_does_not_mutate_definitions_or_use_physical_command_lane(stored_asset):
    original = definition("probe")
    stored_asset.repository.create(original)
    queued_command(stored_asset.commands)

    response = download(stored_asset)

    assert response.success
    assert stored_asset.repository.load("probe") == original
    assert not stored_asset.api.published
    assert not stored_asset.api._static_broadcaster.transforms


@pytest.mark.parametrize(
    "values",
    [
        {"sha256": "../outside"},
        {"sha256": "A" * 64},
        {"sha256": "0" * 64},
        {"byte_count": 0},
        {"byte_count": 100},
        {"offset": 100084},
    ],
)
def test_service_rejects_bad_metadata_without_exposing_local_paths(stored_asset, values):
    response = download(stored_asset, **values)

    assert not response.success
    assert not response.complete
    assert not response.data
    assert str(stored_asset.repository.root_dir) not in response.message


@pytest.mark.parametrize("change", ("tamper", "remove", "symlink"))
def test_service_rejects_changed_asset_between_chunks(stored_asset, tmp_path, change):
    assert download(stored_asset).success
    path = stored_asset.repository.root_dir / "assets" / f"{stored_asset.digest}.stl"
    if change == "tamper":
        path.write_bytes(b"x" + stored_asset.data[1:])
    else:
        path.unlink()
        if change == "symlink":
            outside = tmp_path / "outside.stl"
            outside.write_bytes(stored_asset.data)
            path.symlink_to(outside)

    response = download(stored_asset, offset=MAX_MESH_CHUNK_BYTES)

    assert not response.success
    assert not response.complete
    assert not response.data
    assert str(path) not in response.message
