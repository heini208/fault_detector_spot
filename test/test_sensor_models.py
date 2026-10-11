"""Tests for hand-mounted sensor definitions and transform helpers."""

import math
from dataclasses import replace

import pytest

from fault_detector_spot.shared.geometry.models import (
    PoseData,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    SensorChannel,
    SensorChannelSource,
    SensorDefinition,
    quaternion_from_rpy_degrees,
    rpy_degrees_from_quaternion,
    sensor_definition_from_values,
    sensor_probe_frame,
)
from fault_detector_spot.inspection.model.sensor_geometry import (
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
    SensorHeadGeometry,
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


def test_sensor_definition_round_trips_geometry_separately_from_tip():
    geometry = SensorHeadGeometry(
        collision_sha256="a" * 64,
        collision_byte_count=284,
        hand_to_geometry=PoseData.identity(),
        mounting_template_id=MOUNTING_TEMPLATE_ID,
        mounting_template_revision=MOUNTING_TEMPLATE_REVISION,
        reference_model_fingerprint=REFERENCE_MODEL_FINGERPRINT,
        mounting_template_verified=False,
    )
    definition = sensor_definition_from_values(
        "probe", "Probe", .1, .2, .3, 0, 0, 0, head_geometry=geometry,
    )

    restored = SensorDefinition.from_dict(definition.to_dict())
    restored.validate()

    assert restored == definition
    assert restored.head_geometry.hand_to_geometry != restored.hand_to_probe

    with pytest.raises(ValueError, match="uses stock hand geometry"):
        replace(restored, sensor_id="hand_tip").validate()


def test_manual_sensor_definition_keeps_legacy_serialization():
    definition = SensorDefinition("probe", "Probe", PoseData.identity())

    assert "head_geometry" not in definition.to_dict()
    assert SensorDefinition.from_dict(definition.to_dict()).head_geometry is None
    with pytest.raises(TypeError, match="SensorHeadGeometry"):
        replace(definition, head_geometry={}).validate()


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
        channels=(
            channel,
            SensorChannel(
                channel_id="spot_geometry",
                topic="",
                message_type="",
                source_kind=SensorChannelSource.SPOT_GEOMETRY,
            ),
        ),
    )

    restored = SensorDefinition.from_dict(definition.to_dict())

    assert restored == definition
    assert restored.channels == definition.channels
    legacy_channel = channel.to_dict()
    del legacy_channel["source_kind"]
    assert SensorChannel.from_dict(legacy_channel).source_kind == (
        SensorChannelSource.ROS_TOPIC
    )


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
        (
            SensorChannel(
                "geometry",
                "/tf",
                "tf2_msgs/msg/TFMessage",
                SensorChannelSource.SPOT_GEOMETRY,
            ),
            "must not define a ROS topic",
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


@pytest.mark.parametrize(
    "pitch",
    [90.0, -90.0, 89.999999, -89.999999, 89.9999999, -89.9999999],
)
@pytest.mark.parametrize("roll, yaw", [(30.0, 60.0), (123.0, -87.0)])
def test_rpy_conversion_preserves_rotation_at_gimbal_lock(roll, pitch, yaw):
    original = quaternion_from_rpy_degrees(roll, pitch, yaw)

    angles = rpy_degrees_from_quaternion(original)
    restored = quaternion_from_rpy_degrees(*angles)

    original_values = tuple(original.to_dict().values())
    restored_values = tuple(restored.to_dict().values())
    # Either quaternion sign represents the same rotation.
    sign = 1 if sum(
        before * after
        for before, after in zip(original_values, restored_values)
    ) >= 0 else -1
    assert restored_values == pytest.approx(
        tuple(sign * value for value in original_values), rel=0, abs=1e-8
    )


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
