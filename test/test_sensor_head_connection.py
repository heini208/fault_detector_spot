"""Tests for sensor-head ROS graph discovery and matching."""

from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import (
    SensorAttachmentState,
    SensorHeadConnectionState,
)
from fault_detector_spot.sensing.sensor_head_connection_node import (
    classify_connection,
    discovered_sensor_ids,
    SensorHeadConnectionNode,
)
import pytest
from rclpy.node import NodeNameNonExistentError


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


def connection_monitor(servers):
    """Provide an offline graph whose global list retains a service client."""
    return SimpleNamespace(
        get_node_names_and_namespaces=lambda: list(servers),
        get_service_names_and_types_by_node=Mock(
            side_effect=lambda name, namespace: servers[(name, namespace)],
        ),
        get_service_names_and_types=Mock(return_value=[(
            "/fault_detector/sensors/bmm150_probe/set_acquisition",
            ["fault_detector_msgs/srv/SetSensorAcquisition"],
        )]),
        _fresh_agent_state=Mock(return_value=True),
        _attachment=attachment(
            SensorAttachmentState.STATUS_ACTIVE, active="bmm150_probe",
        ),
        _last_log_signature=None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time),
        ),
        publisher=Mock(),
        get_logger=Mock(),
    )


def test_disconnected_head_is_not_kept_online_by_application_client():
    """A retained client must not mask head or Agent disconnection."""
    servers = {("application_api", "/"): []}
    monitor = connection_monitor(servers)
    servers[("bmm150_probe", "/")] = (
        monitor.get_service_names_and_types.return_value
    )
    assert SensorHeadConnectionNode.poll(monitor).state == (
        SensorHeadConnectionState.STATE_MATCHED
    )

    del servers[("bmm150_probe", "/")]
    disconnected = SensorHeadConnectionNode.poll(monitor)
    assert disconnected.state == SensorHeadConnectionState.STATE_NO_HEADS
    assert disconnected.connected_sensor_ids == []
    monitor.publisher.publish.assert_called_with(disconnected)

    monitor._fresh_agent_state.return_value = False
    assert SensorHeadConnectionNode.poll(monitor).state == (
        SensorHeadConnectionState.STATE_AGENT_UNAVAILABLE
    )
    monitor.get_service_names_and_types.assert_not_called()


def test_node_disappearing_during_poll_does_not_hide_other_heads():
    """Tolerate a graph removal between enumeration and server lookup."""
    monitor = connection_monitor({
        ("disconnected_head", "/"): [], ("bmm150_probe", "/"): [],
    })
    monitor.get_service_names_and_types_by_node.side_effect = [
        NodeNameNonExistentError("Head disconnected"),
        monitor.get_service_names_and_types.return_value,
    ]

    state = SensorHeadConnectionNode.poll(monitor)

    assert state.state == SensorHeadConnectionState.STATE_MATCHED
    assert state.connected_sensor_ids == ["bmm150_probe"]


def test_unrelated_graph_errors_are_not_silenced():
    """Keep unexpected graph failures visible to the caller."""
    monitor = connection_monitor({("bmm150_probe", "/"): []})
    monitor.get_service_names_and_types_by_node.side_effect = RuntimeError(
        "Graph query failed",
    )

    with pytest.raises(RuntimeError, match="Graph query failed"):
        SensorHeadConnectionNode.poll(monitor)
