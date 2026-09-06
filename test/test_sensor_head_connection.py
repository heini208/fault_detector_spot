"""Tests for sensor-head ROS graph discovery and matching."""

from fault_detector_msgs.msg import (
    SensorAttachmentState,
    SensorHeadConnectionState,
)

from fault_detector_spot.sensing.sensor_head_connection_node import (
    classify_connection,
    discovered_sensor_ids,
)


def attachment(status, active="", pending=""):
    """Create one attachment state message for classification tests."""
    message = SensorAttachmentState()
    message.status = status
    message.active_sensor_id = active
    message.pending_sensor_id = pending
    return message


def test_discovery_requires_exact_namespace_and_service_type():
    """Ignore similarly named services and services with the wrong type."""
    services = (
        (
            "/fault_detector/sensors/bmm150_probe/set_acquisition",
            ("fault_detector_msgs/srv/SetSensorAcquisition",),
        ),
        (
            "/fault_detector/sensors/wrong-name/set_acquisition",
            ("fault_detector_msgs/srv/SetSensorAcquisition",),
        ),
        (
            "/fault_detector/sensors/thermal_probe/set_acquisition",
            ("std_srvs/srv/SetBool",),
        ),
    )

    assert discovered_sensor_ids(services) == ("bmm150_probe",)


def test_confirmed_matching_head_is_reported_connected():
    """Match a connected endpoint against the confirmed sensor ID."""
    state, expected, detail = classify_connection(
        True,
        attachment(
            SensorAttachmentState.STATUS_ACTIVE,
            active="bmm150_probe",
        ),
        ("bmm150_probe",),
    )

    assert state == SensorHeadConnectionState.STATE_MATCHED
    assert expected == "bmm150_probe"
    assert "matches" in detail


def test_pending_selection_is_used_for_identity_matching():
    """Prefer the pending ID while physical confirmation is outstanding."""
    state, expected, _ = classify_connection(
        True,
        attachment(
            SensorAttachmentState.STATUS_CONFIRMATION_PENDING,
            active="old_probe",
            pending="bmm150_probe",
        ),
        ("bmm150_probe",),
    )

    assert state == SensorHeadConnectionState.STATE_MATCHED
    assert expected == "bmm150_probe"


def test_connected_head_without_selection_is_unassigned():
    """Expose connected heads for setup when no mount is selected."""
    state, expected, _ = classify_connection(
        True,
        attachment(SensorAttachmentState.STATUS_ACTIVE),
        ("bmm150_probe",),
    )

    assert state == SensorHeadConnectionState.STATE_UNASSIGNED
    assert expected == ""


def test_offline_and_mismatch_are_distinct_states():
    """Distinguish no connected endpoint from a different connected ID."""
    active = attachment(
        SensorAttachmentState.STATUS_ACTIVE,
        active="bmm150_probe",
    )

    offline, _, _ = classify_connection(True, active, ())
    mismatch, _, _ = classify_connection(
        True, active, ("thermal_probe",)
    )

    assert offline == SensorHeadConnectionState.STATE_NO_HEADS
    assert mismatch == SensorHeadConnectionState.STATE_MISMATCH
