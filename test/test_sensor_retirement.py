"""Tests for reference-safe sensor retirement."""

from types import SimpleNamespace

import pytest
import yaml

from fault_detector_spot.inspection.data.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
)
from fault_detector_spot.inspection.data.object_repository import ObjectRepository
from fault_detector_spot.inspection.data.sensor_models import SensorDefinition
from fault_detector_spot.inspection.ros.sensor_registry_node import (
    SensorRegistryNode,
)
from fault_detector_spot.inspection.data.sensor_repository import SensorRepository


def sensor_definition():
    return SensorDefinition(
        sensor_id="bmm150_01",
        display_name="BMM150 Hall sensor",
        hand_to_probe=PoseData.identity(),
    )


def inspection_object(sensor_id="bmm150_01"):
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(tag_id=23, tag_family="36h11"),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id=sensor_id,
            )
        ],
    )


def registry(tmp_path):
    sensors = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    definition = sensor_definition()
    sensors.create(definition)
    objects = ObjectRepository(tmp_path / "objects")
    return SimpleNamespace(
        repository=sensors,
        object_repository=objects,
        _definitions={definition.sensor_id: definition},
    )


def test_saved_routine_blocks_sensor_retirement(tmp_path):
    state = registry(tmp_path)
    state.object_repository.create(inspection_object())

    with pytest.raises(
        ValueError,
        match="motor_a/magnetic_scan",
    ):
        SensorRegistryNode._retire_sensor(state, "bmm150_01")

    assert state.repository.exists("bmm150_01") is True
    assert "bmm150_01" in state._definitions


def test_unreferenced_sensor_is_retired_and_removed_from_registry(tmp_path):
    state = registry(tmp_path)
    state.object_repository.create(inspection_object("other_sensor"))

    retired = SensorRegistryNode._retire_sensor(state, "bmm150_01")

    assert retired.sensor_id == "bmm150_01"
    assert state.repository.exists("bmm150_01") is False
    assert state.repository.is_retired("bmm150_01") is True
    assert "bmm150_01" not in state._definitions


def test_unrelated_incomplete_object_does_not_block_retirement(tmp_path):
    state = registry(tmp_path)
    object_path = state.object_repository.get_object_path("old_object")
    object_path.parent.mkdir(parents=True)
    object_path.write_text(
        yaml.safe_dump({
            "object_id": "old_object",
            "routines": [{
                "routine_id": "old_scan",
                "sensor_id": "other_sensor",
            }],
        }),
        encoding="utf-8",
    )

    retired = SensorRegistryNode._retire_sensor(state, "bmm150_01")

    assert retired.sensor_id == "bmm150_01"
    assert state.repository.is_retired("bmm150_01") is True


def test_incomplete_object_still_blocks_referenced_sensor(tmp_path):
    state = registry(tmp_path)
    object_path = state.object_repository.get_object_path("old_object")
    object_path.parent.mkdir(parents=True)
    object_path.write_text(
        yaml.safe_dump({
            "object_id": "old_object",
            "routines": [{
                "routine_id": "old_scan",
                "sensor_id": "bmm150_01",
            }],
        }),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="old_object/old_scan"):
        SensorRegistryNode._retire_sensor(state, "bmm150_01")

    assert state.repository.exists("bmm150_01") is True
