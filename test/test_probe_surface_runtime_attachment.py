"""Regression tests for close-surface attachment state interpretation."""

from threading import RLock

import pytest

from fault_detector_msgs.msg import SensorAttachmentState

from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource,
)


def surface_source(status, active_sensor_id="", revision=7):
    source = object.__new__(ProbeSurfaceSource)
    source._lock = RLock()
    state = SensorAttachmentState()
    state.status = status
    state.active_sensor_id = active_sensor_id
    state.attachment_revision = revision
    source._attachment_state = state
    return source


def test_confirmed_no_sensor_uses_bare_hand_motion_identity():
    source = surface_source(
        SensorAttachmentState.STATUS_ACTIVE,
        active_sensor_id="",
    )

    sensor_id, revision = source.active_attachment()

    assert sensor_id == BARE_HAND_MOTION_ID
    assert revision == 7


def test_confirmed_sensor_keeps_registered_sensor_identity():
    source = surface_source(
        SensorAttachmentState.STATUS_ACTIVE,
        active_sensor_id="hall_probe",
    )

    sensor_id, revision = source.active_attachment()

    assert sensor_id == "hall_probe"
    assert revision == 7


def test_unconfirmed_no_sensor_state_is_not_motion_ready():
    source = surface_source(
        SensorAttachmentState.STATUS_NO_SENSOR,
    )

    with pytest.raises(ValueError, match="confirmation is pending"):
        source.active_attachment()
