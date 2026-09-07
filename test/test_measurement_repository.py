"""Tests for generic measurement metadata and JSONL persistence."""

import json

import pytest

from fault_detector_spot.inspection.measurement import (
    MeasurementCompletionState,
    MeasurementRecording,
    MeasurementRepository,
)
from fault_detector_spot.inspection.model.sensor_models import SensorChannel


STARTED_AT_NS = 1_788_371_731_438_271_923


def channels():
    return (
        SensorChannel(
            channel_id="magnetic_field",
            topic="/sensors/probe/magnetic_field",
            message_type="sensor_msgs/msg/MagneticField",
        ),
        SensorChannel(
            channel_id="temperature",
            topic="/sensors/probe/temperature",
            message_type="sensor_msgs/msg/Temperature",
        ),
    )


def recording():
    return MeasurementRecording.start(
        object_id="motor_01",
        routine_id="magnetic_scan",
        probe_point_id="bearing_front",
        sensor_id="environmental_probe",
        attachment_revision=4,
        started_at_ns=STARTED_AT_NS,
        configured_channels=channels(),
    )


def test_measurement_requires_at_least_one_configured_channel():
    with pytest.raises(ValueError, match="at least one channel"):
        MeasurementRecording.start(
            object_id="motor_01",
            routine_id="magnetic_scan",
            probe_point_id="bearing_front",
            sensor_id="offline_probe",
            attachment_revision=4,
            started_at_ns=STARTED_AT_NS,
            configured_channels=(),
        )


def test_repository_writes_channels_and_final_metadata(tmp_path):
    repository = MeasurementRepository(tmp_path)
    active = recording()
    repository.create(active)

    magnetic_path = repository.get_channel_path(
        active,
        "magnetic_field",
    )
    temperature_path = repository.get_channel_path(
        active,
        "temperature",
    )
    expected_prefix = (
        tmp_path
        / "motor_01"
        / "magnetic_scan"
        / "bearing_front"
        / "2026-09-02"
        / "2026-09-02T17-55-31.438271923Z"
    )
    assert magnetic_path == expected_prefix / "magnetic_field.jsonl"
    assert temperature_path == expected_prefix / "temperature.jsonl"
    assert repository.get_metadata_path(active) == (
        expected_prefix / "metadata.json"
    )

    repository.append_sample(
        active,
        "magnetic_field",
        {"source_time_ns": 10, "data": {"x": 1.5}},
    )
    repository.append_sample(
        active,
        "temperature",
        {"source_time_ns": None, "data": {"value": 21.0}},
    )
    finalized = repository.finalize(
        active,
        MeasurementCompletionState.COMPLETE,
        STARTED_AT_NS + 2_000_000_000,
    )

    assert not repository.is_open(active)
    assert finalized.sample_counts == {
        "magnetic_field": 1,
        "temperature": 1,
    }
    assert json.loads(magnetic_path.read_text(encoding="utf-8")) == {
        "source_time_ns": 10,
        "data": {"x": 1.5},
    }
    restored = repository.load(
        object_id="motor_01",
        routine_id="magnetic_scan",
        probe_point_id="bearing_front",
        sensor_id="environmental_probe",
        started_at_ns=STARTED_AT_NS,
    )
    assert restored == finalized
    assert restored.configured_channels == channels()


def test_repository_rejects_recording_identity_collision(tmp_path):
    active = recording()
    owner = MeasurementRepository(tmp_path)
    owner.create(active)

    with pytest.raises(FileExistsError):
        MeasurementRepository(tmp_path).create(recording())

    owner.finalize(
        active,
        MeasurementCompletionState.CANCELLED,
        STARTED_AT_NS,
    )

    metadata_only_root = tmp_path / "metadata_only"
    metadata_only_repository = MeasurementRepository(metadata_only_root)
    metadata_path = metadata_only_repository.get_metadata_path(recording())
    metadata_path.parent.mkdir(parents=True)
    metadata_path.write_text("existing", encoding="utf-8")

    with pytest.raises(FileExistsError):
        metadata_only_repository.create(recording())

    assert metadata_path.read_text(encoding="utf-8") == "existing"


@pytest.mark.parametrize(
    ("context", "directories"),
    (
        (("", "", ""), ("manual",)),
        (("motor_01", "", ""), ("motor_01",)),
        (
            ("motor_01", "magnetic_scan", ""),
            ("motor_01", "magnetic_scan"),
        ),
    ),
)
def test_optional_context_uses_only_present_directories(
    tmp_path,
    context,
    directories,
):
    repository = MeasurementRepository(tmp_path)
    active = MeasurementRecording.start(
        object_id=context[0],
        routine_id=context[1],
        probe_point_id=context[2],
        sensor_id="environmental_probe",
        attachment_revision=4,
        started_at_ns=STARTED_AT_NS,
        configured_channels=channels(),
    )

    assert repository.get_metadata_path(active) == (
        tmp_path.joinpath(*directories)
        / "2026-09-02"
        / "2026-09-02T17-55-31.438271923Z"
        / "metadata.json"
    )


@pytest.mark.parametrize(
    "context",
    (("", "scan", ""), ("motor", "", "bearing")),
)
def test_optional_context_rejects_hierarchy_gaps(context):
    with pytest.raises(ValueError, match="requires"):
        MeasurementRecording.start(
            object_id=context[0],
            routine_id=context[1],
            probe_point_id=context[2],
            sensor_id="environmental_probe",
            attachment_revision=4,
            started_at_ns=STARTED_AT_NS,
            configured_channels=channels(),
        )
