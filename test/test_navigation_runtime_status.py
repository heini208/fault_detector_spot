"""Runtime status transport expiry and context notification behavior."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from fault_detector_msgs.msg import NavigationSetupState

from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
import fault_detector_spot.application.api.navigation_setup_api as module
from test_command_request_correlation import FakeClock
from test_navigation_setup_coordinator import coordinator


def test_runtime_feed_expires_and_keeps_mode_unknown(monkeypatch):
    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.coordinator = Mock()
    api._runtime_received_at = None
    now = [1.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    api._receive_runtime(DiagnosticStatus(
        level=DiagnosticStatus.ERROR, message="Nav2 is not running",
        values=[KeyValue(key="mode", value="localization"),
                KeyValue(key="active_map", value="plant")],
    ))
    api.coordinator.observe_runtime.assert_called_once_with(
        "localization", "plant", "Nav2 is not running",
    )
    now[0] = 2.0
    api._check_runtime_status()
    assert api.coordinator.observe_runtime.call_count == 1
    now[0] = 3.1
    api._check_runtime_status()
    api.coordinator.observe_runtime.assert_called_with(None, "", "Runtime status unavailable")


@pytest.mark.parametrize("runtime_error", ["", "Nav2 is not running", "Runtime status unavailable"])
def test_setup_state_transports_runtime_health_separately_from_transaction_failure(
    tmp_path, runtime_error,
):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(context, "plant").context
    navigation.observe_runtime("localization", "plant")
    if runtime_error:
        navigation.observe_runtime(
            None if "unavailable" in runtime_error else "localization",
            "plant", runtime_error,
        )
    context = navigation.context(context.context_id, context.client_id)
    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.node = SimpleNamespace(get_clock=lambda: FakeClock())

    state = api._state(
        navigation.snapshot(context), 0, NavigationSetupState.STATE_FAILED,
        "Waypoint could not be saved",
    )

    assert state.mode == NavigationSetupState.MODE_LOCALIZATION
    assert state.active_map == "plant"
    assert state.runtime_error == runtime_error
    assert state.detail.startswith("Waypoint could not be saved")


def test_contextless_failure_does_not_claim_runtime_is_stopped():
    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.node = SimpleNamespace(get_clock=lambda: FakeClock())
    api._state_publisher = Mock()
    goal = SimpleNamespace(
        client_id="navigation-ui", context_id="expired-context",
        intent=SimpleNamespace(operation=0),
    )
    result = api._abort(Mock(), goal, "Navigation setup context expired")
    assert result.state.state == NavigationSetupState.STATE_FAILED
    assert result.state.runtime_error == "Runtime status unavailable"
