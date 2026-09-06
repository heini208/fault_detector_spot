"""Tests for hand-mounted sensor definitions and transform helpers."""

import math

import pytest

from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    SensorChannel,
    SensorDefinition,
    quaternion_from_rpy_degrees,
    rpy_degrees_from_quaternion,
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


def test_sensor_definition_round_trips_configured_channels():
    channel = SensorChannel(
        channel_id="magnetic_field",
        topic="/sensors/bmm150_probe/magnetic_field",
        message_type="sensor_msgs/msg/MagneticField",
    )
    definition = sensor_definition_from_values(
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

    restored = SensorDefinition.from_dict(definition.to_dict())

    assert restored == definition
    assert restored.channels == (channel,)


def test_legacy_sensor_definition_without_channels_loads_empty():
    definition = sensor_definition_from_values(
        "legacy_probe",
        "Legacy probe",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    data = definition.to_dict()
    del data["channels"]

    assert SensorDefinition.from_dict(data).channels == ()


def test_duplicate_channel_ids_are_rejected():
    channel = SensorChannel(
        "field",
        "/sensors/probe/field",
        "sensor_msgs/msg/MagneticField",
    )
    definition = SensorDefinition(
        sensor_id="probe",
        display_name="Probe",
        hand_to_probe=PoseData.identity(),
        channels=(channel, channel),
    )

    with pytest.raises(ValueError, match="Duplicate channel ID"):
        definition.validate()


@pytest.mark.parametrize(
    ("channel", "message"),
    (
        (SensorChannel("", "/topic", "std_msgs/msg/String"), "channel ID"),
        (SensorChannel("value", "", "std_msgs/msg/String"), "topic"),
        (
            SensorChannel("value", "relative/topic", "std_msgs/msg/String"),
            "absolute ROS topic",
        ),
        (SensorChannel("value", "/topic", ""), "message type"),
        (
            SensorChannel("value", "/topic", "std_msgs/String"),
            "package/msg/Type",
        ),
        (
            SensorChannel(
                "../value",
                "/topic",
                "std_msgs/msg/String",
            ),
            "channel ID",
        ),
    ),
)
def test_invalid_sensor_channels_are_rejected(channel, message):
    with pytest.raises(ValueError, match=message):
        channel.validate()


def test_short_test_sensor_id_is_valid():
    definition = sensor_definition_from_values(
        "test",
        "Test sensor",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    assert definition.sensor_id == "test"
    assert definition.probe_frame == "test_probe"


def test_bare_hand_motion_uses_real_hand_frame_without_sensor_definition():
    assert BARE_HAND_MOTION_ID == "hand"
    assert sensor_probe_frame(BARE_HAND_MOTION_ID) == "hand"

    definition = SensorDefinition(
        sensor_id=BARE_HAND_MOTION_ID,
        display_name="Not allowed",
        hand_to_probe=PoseData.identity(),
    )

    with pytest.raises(ValueError, match="reserved"):
        definition.validate()


def test_rpy_degree_conversion_round_trips_quaternion():
    quaternion = quaternion_from_rpy_degrees(12.0, -7.0, 91.0)

    roll, pitch, yaw = rpy_degrees_from_quaternion(quaternion)

    assert roll == pytest.approx(12.0)
    assert pitch == pytest.approx(-7.0)
    assert yaw == pytest.approx(91.0)


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
