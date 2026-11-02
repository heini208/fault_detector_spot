"""Validate reference-capture shutdown and transport ownership."""

import inspect
from unittest.mock import Mock

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)
from fault_detector_spot.application.api.probe_reference_capture_api import (
    ProbeReferenceCaptureApi,
)
from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
)
from fault_detector_spot.ui.ros.probe_setup_client import ProbeSetupClient


def test_reference_capture_api_shutdown_stops_work_before_destroying_server():
    events = []
    api = ProbeReferenceCaptureApi.__new__(ProbeReferenceCaptureApi)
    api.capture_coordinator = Mock()
    api._action_server = Mock()
    api.capture_coordinator.close.side_effect = lambda: events.append("capture")
    api._action_server.destroy.side_effect = lambda: events.append("action")
    api.close()
    api.close()
    assert events == ["capture", "action"]


def test_reference_capture_coordinator_shutdown_interrupts_waits():
    source = inspect.getsource(ProbeReferenceCaptureCoordinator.close)
    assert "request_shutdown()" in source
    assert "_shutdown.set()" in inspect.getsource(
        ProbeReferenceCaptureCoordinator.request_shutdown
    )
    assert "unregister" in source


def test_application_api_node_destroy_uses_the_ordered_shutdown_owner():
    node = Mock()
    assert ApplicationApiNode.destroy_node(node) is node.shutdown.return_value
    node.shutdown.assert_called_once_with()


def test_probe_setup_client_blocks_close_while_capture_is_active():
    source = inspect.getsource(ProbeSetupClient.close)
    assert "_capture_goal_handles" in source


def test_probe_setup_transactions_are_not_blocked_during_capture():
    source = inspect.getsource(ProbeSetupClient._send)
    assert "_capture_goal_handles" not in source
