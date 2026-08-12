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


def test_mapping_start_projects_mapping_while_request_is_inflight():
    message = _state(
        NavigationSetupIntent.OPERATION_START_MAPPING,
        NavigationSetupState.STATE_RUNNING,
        NavigationSetupState.MODE_NONE,
    )

    assert (
        NavigationSetupClient._display_mode(message)
        == NavigationSetupState.MODE_MAPPING
    )


def test_localization_start_projects_localization_while_inflight():
    message = _state(
        NavigationSetupIntent.OPERATION_START_LOCALIZATION,
        NavigationSetupState.STATE_QUEUED,
        NavigationSetupState.MODE_NONE,
    )

    assert (
        NavigationSetupClient._display_mode(message)
        == NavigationSetupState.MODE_LOCALIZATION
    )


def test_stop_projects_none_while_request_is_inflight():
    message = _state(
        NavigationSetupIntent.OPERATION_STOP_MAPPING,
        NavigationSetupState.STATE_RUNNING,
        NavigationSetupState.MODE_MAPPING,
    )

    assert (
        NavigationSetupClient._display_mode(message)
        == NavigationSetupState.MODE_NONE
    )


def test_failed_start_reverts_to_authoritative_mode():
    message = _state(
        NavigationSetupIntent.OPERATION_START_MAPPING,
        NavigationSetupState.STATE_FAILED,
        NavigationSetupState.MODE_NONE,
    )

    assert (
        NavigationSetupClient._display_mode(message)
        == NavigationSetupState.MODE_NONE
    )


def test_failed_stop_reverts_to_authoritative_running_mode():
    message = _state(
        NavigationSetupIntent.OPERATION_STOP_MAPPING,
        NavigationSetupState.STATE_FAILED,
        NavigationSetupState.MODE_MAPPING,
    )

    assert (
        NavigationSetupClient._display_mode(message)
        == NavigationSetupState.MODE_MAPPING
    )


def test_state_fingerprint_contains_visible_navigation_state():
    source = inspect.getsource(NavigationSetupClient._emit_state)

    assert "int(state.mode)" in source
    assert "state.active_map" in source
    assert "tuple(state.waypoint_names)" in source
    assert "tuple(state.landmark_names)" in source
