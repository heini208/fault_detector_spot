"""Tests for persistent sensor calibration storage."""

import pytest
import yaml

from fault_detector_spot.inspection.model.sensor_models import (
    NO_SENSOR_MOUNT_ID,
    SensorDefinition,
    sensor_definition_from_values,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)


def definition(sensor_id="bmm150_01"):
    return sensor_definition_from_values(
        sensor_id,
        "BMM150 Hall sensor",
        0.20,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def test_builtin_no_sensor_mount_is_always_available(tmp_path):
    repository = SensorRepository(tmp_path)

    built_in = repository.load(NO_SENSOR_MOUNT_ID)

    assert repository.exists(NO_SENSOR_MOUNT_ID) is True
    assert repository.list_sensor_ids() == [NO_SENSOR_MOUNT_ID]
    assert built_in.hand_to_probe.position.x == pytest.approx(0.0)
    assert built_in.hand_to_probe.position.y == pytest.approx(0.0)
    assert built_in.hand_to_probe.position.z == pytest.approx(0.0)
    assert built_in.hand_to_probe.orientation.w == pytest.approx(1.0)
    assert not repository.get_sensor_path(NO_SENSOR_MOUNT_ID).exists()


def test_builtin_no_sensor_mount_cannot_be_replaced_or_retired(tmp_path):
    repository = SensorRepository(tmp_path)
    replacement = SensorDefinition(
        sensor_id=NO_SENSOR_MOUNT_ID,
        display_name="Replacement",
        hand_to_probe=definition().hand_to_probe,
    )

    with pytest.raises(ValueError, match="cannot be replaced"):
        repository.create(replacement)
    with pytest.raises(ValueError, match="cannot be retired"):
        repository.retire(NO_SENSOR_MOUNT_ID)

    assert repository.load(NO_SENSOR_MOUNT_ID).display_name == (
        "No Sensor Mount"
    )


def test_create_load_and_list_sensor_definition(tmp_path):
    repository = SensorRepository(tmp_path)
    stored = definition()

    repository.create(stored)

    assert repository.list_sensor_ids() == [
        NO_SENSOR_MOUNT_ID,
        "bmm150_01",
    ]
    assert repository.load("bmm150_01") == stored
    data = yaml.safe_load(
        repository.get_sensor_path("bmm150_01").read_text(
            encoding="utf-8"
        )
    )
    assert data["sensor_id"] == "bmm150_01"
    assert data["hand_to_probe"]["position"]["x"] == pytest.approx(0.20)
    assert "probe_frame" not in data


def test_duplicate_sensor_id_cannot_be_overwritten(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition())

    with pytest.raises(FileExistsError, match="already exists"):
        repository.create(definition())

    assert repository.load("bmm150_01").hand_to_probe.position.x == (
        pytest.approx(0.20)
    )


def test_sensor_file_id_must_match_filename(tmp_path):
    repository = SensorRepository(tmp_path)
    path = repository.get_sensor_path("bmm150_01")
    path.parent.mkdir(parents=True, exist_ok=True)
    data = definition("other_mount").to_dict()
    path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="ID mismatch"):
        repository.load("bmm150_01")


def test_load_all_keeps_builtin_first_then_sorts_custom_ids(tmp_path):
    repository = SensorRepository(tmp_path)
    repository.create(definition("sensor_b"))
    repository.create(definition("sensor_a"))

    assert [
        item.sensor_id for item in repository.load_all()
    ] == [NO_SENSOR_MOUNT_ID, "sensor_a", "sensor_b"]


def test_retire_moves_definition_and_permanently_reserves_id(tmp_path):
    repository = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    repository.create(definition("retired_mount"))

    retired = repository.retire("retired_mount")

    assert retired.sensor_id == "retired_mount"
    assert repository.list_sensor_ids() == [NO_SENSOR_MOUNT_ID]
    assert repository.list_retired_sensor_ids() == ["retired_mount"]
    assert repository.is_retired("retired_mount") is True
    assert repository.get_retired_sensor_path(
        "retired_mount"
    ).is_file()
    with pytest.raises(FileExistsError, match="cannot be reused"):
        repository.create(definition("retired_mount"))
    with pytest.raises(FileNotFoundError, match="already retired"):
        repository.retire("retired_mount")
