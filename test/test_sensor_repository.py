"""Tests for persistent physical sensor transform storage."""

from dataclasses import replace
import hashlib
import struct

import pytest
import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    SensorChannel,
    SensorDefinition,
    sensor_definition_from_values,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.inspection.model.sensor_geometry import (
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
    SensorHeadGeometry,
)
from fault_detector_spot.shared.geometry.models import PoseData


def definition(sensor_id="bmm150_01", x=0.20):
    return sensor_definition_from_values(
        sensor_id,
        "BMM150 Hall sensor",
        x,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def test_empty_repository_has_no_fake_no_sensor_definition(tmp_path):
    repository = SensorRepository(tmp_path)

    assert repository.list_sensor_ids() == []
    assert repository.load_all() == []
    assert not repository.get_sensor_path("hand").exists()


def test_reserved_hand_id_cannot_be_created(tmp_path):
    repository = SensorRepository(tmp_path)
    reserved = SensorDefinition(
        sensor_id="hand",
        display_name="Fake hand sensor",
        hand_to_probe=definition().hand_to_probe,
    )

    with pytest.raises(ValueError, match="reserved"):
        repository.create(reserved)

    assert repository.list_sensor_ids() == []


def test_create_load_and_list_sensor_definition(tmp_path):
    repository = SensorRepository(tmp_path)
    stored = definition("test")

    repository.create(stored)

    assert repository.list_sensor_ids() == ["test"]
    assert repository.load("test") == stored
    data = yaml.safe_load(
        repository.get_sensor_path("test").read_text(encoding="utf-8")
    )
    assert data["sensor_id"] == "test"
    assert data["hand_to_probe"]["position"]["x"] == pytest.approx(0.20)
    assert "probe_frame" not in data


def test_repository_persists_sensor_channels_in_mount_yaml(tmp_path):
    repository = SensorRepository(tmp_path)
    channel = SensorChannel(
        channel_id="magnetic_field",
        topic="/sensors/bmm150_probe/magnetic_field",
        message_type="sensor_msgs/msg/MagneticField",
    )
    stored = sensor_definition_from_values(
        "bmm150_probe",
        "BMM150 probe",
        0.2,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        channels=(channel,),
    )

    repository.create(stored)

    assert repository.load("bmm150_probe").channels == (channel,)
    data = yaml.safe_load(
        repository.get_sensor_path("bmm150_probe").read_text(
            encoding="utf-8"
        )
    )
    assert data["channels"] == [channel.to_dict()]


def test_create_rejects_existing_active_sensor_id(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition())

    with pytest.raises(FileExistsError, match="already exists"):
        repository.create(definition(x=0.30))

    assert repository.load("bmm150_01").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_update_replaces_existing_transform_atomically(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition("test", x=0.10))

    updated = repository.update(definition("test", x=0.35))

    assert updated.hand_to_probe.position.x == pytest.approx(0.35)
    assert repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.35)
    )


def test_update_requires_existing_sensor(tmp_path):
    repository = SensorRepository(tmp_path)

    with pytest.raises(FileNotFoundError, match="does not exist"):
        repository.update(definition("missing"))


def test_delete_removes_sensor_and_allows_same_id_to_be_recreated(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition("test", x=0.10))

    deleted = repository.delete("test")
    repository.create(definition("test", x=0.40))

    assert deleted.sensor_id == "test"
    assert repository.list_sensor_ids() == ["test"]
    assert repository.load("test").hand_to_probe.position.x == (
        pytest.approx(0.40)
    )


def test_sensor_file_id_must_match_filename(tmp_path):
    repository = SensorRepository(tmp_path)
    path = repository.get_sensor_path("bmm150_01")
    path.parent.mkdir(parents=True, exist_ok=True)
    data = definition("other_mount").to_dict()
    path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="ID mismatch"):
        repository.load("bmm150_01")


def test_load_all_sorts_physical_sensor_ids(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition("sensor_b"))
    repository.create(definition("sensor_a"))

    assert [
        item.sensor_id for item in repository.load_all()
    ] == ["sensor_a", "sensor_b"]


@pytest.fixture
def collision_asset():
    vertices = ((0, 0, 0), (0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    data = bytes(80) + struct.pack("<I", len(faces))
    for face in faces:
        points = [coordinate for index in face for coordinate in vertices[index]]
        data += struct.pack("<12fH", 0, 0, 0, *points, 0)
    geometry = SensorHeadGeometry(
        hashlib.sha256(data).hexdigest(), len(data), PoseData.identity(),
        MOUNTING_TEMPLATE_ID, MOUNTING_TEMPLATE_REVISION,
        REFERENCE_MODEL_FINGERPRINT, False,
    )
    return data, geometry


def test_geometry_and_pose_survive_source_deletion_and_repository_restart(
    tmp_path, collision_asset,
):
    data, geometry = collision_asset
    source = tmp_path / "download.stl"
    source.write_bytes(data)
    repository = SensorRepository(tmp_path / "sensors")
    repository.store_collision_chunk(
        geometry.collision_sha256, len(data), 0, source.read_bytes(),
    )
    stored = replace(definition("test", x=0.15), head_geometry=geometry)
    repository.create(stored)
    source.unlink()

    restarted = SensorRepository(repository.root_dir)

    assert restarted.load("test") == stored
    assert restarted.read_collision_mesh(geometry) == data
    assert (repository.root_dir / geometry.asset_path).read_bytes() == data
    assert str(source) not in repository.get_sensor_path("test").read_text()


def test_deleting_definition_keeps_asset_shared_by_another_mount(
    tmp_path, collision_asset,
):
    data, geometry = collision_asset
    repository = SensorRepository(tmp_path)
    repository.store_collision_chunk(geometry.collision_sha256, len(data), 0, data)
    repository.create(replace(definition("first"), head_geometry=geometry))
    second = replace(definition("second"), head_geometry=geometry)
    repository.create(second)

    repository.delete("first")

    assert repository.load("second") == second
    assert repository.read_collision_mesh(geometry) == data
    assert len(list((tmp_path / "assets").iterdir())) == 1


def test_missing_asset_rejects_create_without_writing_yaml(tmp_path, collision_asset):
    _, geometry = collision_asset
    repository = SensorRepository(tmp_path)

    with pytest.raises(FileNotFoundError):
        repository.create(replace(definition("test"), head_geometry=geometry))

    assert not repository.get_sensor_path("test").exists()


def test_failed_geometry_update_preserves_previous_pose_and_geometry(
    tmp_path, collision_asset,
):
    data, geometry = collision_asset
    repository = SensorRepository(tmp_path)
    repository.store_collision_chunk(geometry.collision_sha256, len(data), 0, data)
    original = replace(definition("test", x=0.15), head_geometry=geometry)
    repository.create(original)
    before = repository.get_sensor_path("test").read_bytes()
    missing = replace(geometry, collision_sha256="0" * 64)

    with pytest.raises(FileNotFoundError):
        repository.update(replace(definition("test", x=0.9), head_geometry=missing))

    assert repository.get_sensor_path("test").read_bytes() == before
    assert repository.load("test") == original


def test_atomic_yaml_failure_preserves_previous_pose_and_geometry(
    tmp_path, collision_asset, monkeypatch,
):
    data, geometry = collision_asset
    repository = SensorRepository(tmp_path)
    repository.store_collision_chunk(geometry.collision_sha256, len(data), 0, data)
    original = replace(definition("test", x=0.15), head_geometry=geometry)
    repository.create(original)
    before = repository.get_sensor_path("test").read_bytes()

    def fail_replace(*_args):
        raise OSError("simulated disk failure")

    monkeypatch.setattr(
        "fault_detector_spot.shared.persistence.file_storage.os.replace",
        fail_replace,
    )
    with pytest.raises(OSError, match="disk failure"):
        repository.update(replace(definition("test", x=0.9), head_geometry=geometry))

    assert repository.get_sensor_path("test").read_bytes() == before
    assert repository.load("test") == original
    assert sorted(path.name for path in tmp_path.iterdir()) == ["assets", "test.yaml"]


def test_tampered_asset_rejects_load_and_update(tmp_path, collision_asset):
    data, geometry = collision_asset
    repository = SensorRepository(tmp_path)
    repository.store_collision_chunk(geometry.collision_sha256, len(data), 0, data)
    original = replace(definition("test"), head_geometry=geometry)
    repository.create(original)
    before = repository.get_sensor_path("test").read_bytes()
    (tmp_path / geometry.asset_path).write_bytes(b"invalid")

    with pytest.raises(ValueError):
        repository.load("test")
    with pytest.raises(ValueError):
        repository.update(replace(original, display_name="Changed"))

    assert repository.get_sensor_path("test").read_bytes() == before
