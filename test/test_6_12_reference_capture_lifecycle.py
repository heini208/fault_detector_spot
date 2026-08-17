"""Validate reference-capture shutdown and transport ownership."""

import inspect

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
    source = inspect.getsource(ProbeReferenceCaptureApi.close)
    assert source.index("capture_coordinator.close()") < source.index(
        "_action_server.destroy()"
    )


def test_reference_capture_coordinator_shutdown_interrupts_waits():
    source = inspect.getsource(ProbeReferenceCaptureCoordinator.close)
    assert "_shutdown.set()" in source
    assert "unregister" in source


def test_application_api_node_destroys_reference_capture_api():
    source = inspect.getsource(ApplicationApiNode.destroy_node)
    assert "probe_reference_capture_api.close()" in source


def test_probe_setup_client_blocks_close_while_capture_is_active():
    source = inspect.getsource(ProbeSetupClient.close)
    assert "_capture_goal_handles" in source


def test_probe_setup_transactions_are_not_blocked_during_capture():
    source = inspect.getsource(ProbeSetupClient._send)
    assert "_capture_goal_handles" not in source
