"""Tests for persistent physical sensor transform storage."""

import pytest
import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
    sensor_definition_from_values,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)


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


def test_legacy_retired_id_no_longer_blocks_reuse(tmp_path):
    repository = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    legacy_path = repository.get_retired_sensor_path("test")
    legacy_path.parent.mkdir(parents=True, exist_ok=True)
    legacy_path.write_text(
        yaml.safe_dump(definition("test").to_dict()),
        encoding="utf-8",
    )

    repository.create(definition("test"))

    assert repository.exists("test") is True
    assert repository.is_retired("test") is False


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
