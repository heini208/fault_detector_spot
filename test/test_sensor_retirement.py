"""Tests for attachment-safe sensor deletion."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.inspection.model.models import PoseData
from fault_detector_spot.inspection.model.sensor_models import SensorDefinition
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.inspection.ros.sensor_registry_node import (
    SensorRegistryNode,
)


def sensor_definition():
    return SensorDefinition(
        sensor_id="bmm150_01",
        display_name="BMM150 Hall sensor",
        hand_to_probe=PoseData.identity(),
    )


def registry(tmp_path):
    sensors = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    definition = sensor_definition()
    sensors.create(definition)
    state = SimpleNamespace(
        repository=sensors,
        _definitions={definition.sensor_id: definition},
        _attachment_state=SimpleNamespace(
            active_sensor_id="",
            pending_sensor_id="",
        ),
    )

    def require_mutation_allowed(sensor_id):
        return SensorRegistryNode._require_mutation_allowed(
            state,
            sensor_id,
        )

    state._require_mutation_allowed = require_mutation_allowed
    return state


def test_sensor_deletion_has_no_inspection_repository_dependency(tmp_path):
    state = registry(tmp_path)

    deleted = SensorRegistryNode._delete_sensor(state, "bmm150_01")
    state.repository.create(sensor_definition())

    assert deleted.sensor_id == "bmm150_01"
    assert state.repository.exists("bmm150_01") is True
    assert state.repository.is_retired("bmm150_01") is False
    assert "bmm150_01" not in state._definitions
    assert not hasattr(state, "object_repository")


def test_selected_sensor_cannot_be_deleted(tmp_path):
    state = registry(tmp_path)
    state._attachment_state.active_sensor_id = "bmm150_01"

    with pytest.raises(RuntimeError, match="currently selected"):
        SensorRegistryNode._delete_sensor(state, "bmm150_01")

    assert state.repository.exists("bmm150_01") is True
