"""Runtime status transport expiry and context notification behavior."""

from unittest.mock import Mock
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
import fault_detector_spot.application.api.navigation_setup_api as module


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
