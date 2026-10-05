"""Tests for navigation client rendering safety."""

import inspect

from fault_detector_msgs.msg import (
    NavigationSetupIntent,
    NavigationSetupState,
)

from fault_detector_spot.ui.ros.navigation_setup_client import (
    NavigationSetupClient,
)


def _state(operation, state_code, mode):
    message = NavigationSetupState()
    message.operation = operation
    message.state = state_code
    message.mode = mode
    return message


def test_runtime_state_preserves_last_known_map_list():
    source = inspect.getsource(NavigationSetupClient._emit_state)

    assert "_last_map_names" in source
    assert "_RUNTIME_OPERATIONS" in source
    assert "state.map_names = list(self._last_map_names)" in source


def test_inflight_and_failed_requests_keep_observed_mode(monkeypatch):
    from unittest.mock import Mock
    import fault_detector_spot.ui.ros.navigation_setup_client as module
    monkeypatch.setattr(module, "ActionClient", Mock())
    client = NavigationSetupClient(Mock(), "ui")
    received = []
    client.state_changed.connect(received.append)
    for operation in (
        NavigationSetupIntent.OPERATION_START_MAPPING,
        NavigationSetupIntent.OPERATION_START_LOCALIZATION,
        NavigationSetupIntent.OPERATION_STOP_MAPPING,
    ):
        for code in (NavigationSetupState.STATE_RUNNING, NavigationSetupState.STATE_FAILED):
            for mode in (NavigationSetupState.MODE_NONE, NavigationSetupState.MODE_MAPPING):
                message = _state(operation, code, mode)
                message.client_id = "ui"
                message.context_id = "context"
                message.revision = len(received) + 1
                client._emit_state(message)
                assert received[-1].mode == mode


def test_state_fingerprint_contains_visible_navigation_state():
    source = inspect.getsource(NavigationSetupClient._emit_state)

    assert "int(state.mode)" in source
    assert "state.active_map" in source
    assert "tuple(state.waypoint_names)" in source
    assert "tuple(state.landmark_names)" in source
