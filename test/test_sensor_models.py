"""Tests for immutable hand-mounted sensor definitions."""

import math

import pytest

from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.model.sensor_models import (
    SensorDefinition,
    quaternion_from_rpy_degrees,
    sensor_definition_from_values,
    sensor_probe_frame,
)


def test_sensor_definition_round_trip_and_derived_frame():
    definition = sensor_definition_from_values(
        "bmm150_01",
        "BMM150 Hall sensor",
        0.20,
        -0.01,
        0.03,
        0.0,
        0.0,
        90.0,
    )

    restored = SensorDefinition.from_dict(definition.to_dict())
    restored.validate()

    assert restored == definition
    assert restored.probe_frame == "bmm150_01_probe"
    assert sensor_probe_frame("bmm150_01") == "bmm150_01_probe"


def test_rpy_degrees_are_saved_as_normalized_quaternion():
    quaternion = quaternion_from_rpy_degrees(0.0, 0.0, 90.0)

    assert quaternion.x == pytest.approx(0.0)
    assert quaternion.y == pytest.approx(0.0)
    assert quaternion.z == pytest.approx(math.sqrt(0.5))
    assert quaternion.w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize(
    "sensor_id",
    ["", "../sensor", "a/b", "sensor mount", "sensor-1", "Sensor1"],
)
def test_invalid_sensor_ids_are_rejected(sensor_id):
    definition = SensorDefinition(
        sensor_id=sensor_id,
        display_name="Sensor",
        hand_to_probe=PoseData.identity(),
    )

    with pytest.raises(ValueError):
        definition.validate()


def test_non_finite_translation_is_rejected():
    definition = SensorDefinition(
        sensor_id="bmm150_01",
        display_name="BMM150 Hall sensor",
        hand_to_probe=PoseData.identity(),
    )
    definition.hand_to_probe.position = Vector3Data(
        x=float("nan"),
        y=0.0,
        z=0.0,
    )

    with pytest.raises(ValueError, match="non-finite"):
        definition.validate()
