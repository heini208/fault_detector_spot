"""Tests for persisted physical sensor selection."""

import pytest

from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    PersistedSensorAttachmentSelection,
    SensorAttachmentStateStore,
)


def test_missing_state_represents_first_startup(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")

    assert store.load() is None


def test_sensor_selection_round_trip(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")
    selection = PersistedSensorAttachmentSelection(
        sensor_id="bmm150_01",
        attachment_revision=7,
    )

    store.save(selection)

    assert store.load() == selection


def test_no_sensor_selection_round_trip(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")
    selection = PersistedSensorAttachmentSelection(
        sensor_id="",
        attachment_revision=3,
    )

    store.save(selection)

    assert store.load() == selection


def test_clear_removes_persisted_selection(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")
    store.save(
        PersistedSensorAttachmentSelection(
            sensor_id="bmm150_01",
            attachment_revision=4,
        )
    )

    store.clear()

    assert store.load() is None


def test_invalid_attachment_revision_is_rejected(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")

    with pytest.raises(ValueError):
        store.save(
            PersistedSensorAttachmentSelection(
                sensor_id="bmm150_01",
                attachment_revision=-1,
            )
        )


def test_invalid_sensor_id_is_rejected(tmp_path):
    store = SensorAttachmentStateStore(tmp_path / "attachment.yaml")

    with pytest.raises(ValueError):
        store.save(
            PersistedSensorAttachmentSelection(
                sensor_id="../sensor",
                attachment_revision=1,
            )
        )
