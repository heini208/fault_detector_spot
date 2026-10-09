"""Localization capture failures remain actionable at the waypoint API boundary."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.action import ExecuteNavigationSetup
from fault_detector_msgs.msg import NavigationSetupIntent, NavigationSetupState

from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
from fault_detector_spot.application.coordinators.navigation_setup_coordinator import MODE_LOCALIZATION
from test_command_request_correlation import FakeClock
from test_navigation_setup_coordinator import coordinator


@pytest.mark.parametrize("detail", [
    "Localization estimate is stale: no new estimate for 1.80 s",
    "Current map-to-base_link transform is unavailable",
])
def test_pose_capture_error_reaches_ui_without_saving_waypoint(tmp_path, detail):
    navigation, commands = coordinator(tmp_path)
    state = navigation.open_context("navigation-ui")
    state = navigation.create_map_definition(state.context, "plant")
    navigation.observe_runtime(MODE_LOCALIZATION, "plant")
    context = navigation.context(state.context.context_id, "navigation-ui")
    navigation.current_pose = Mock(side_effect=ValueError(detail))
    path = navigation.map_repository.get_map_path("plant")
    before = path.read_text()

    api = NavigationSetupApi.__new__(NavigationSetupApi)
    api.coordinator = navigation
    api.node = SimpleNamespace(get_clock=lambda: FakeClock())
    api._state_publisher = Mock()
    operation = NavigationSetupIntent.OPERATION_ADD_CURRENT_WAYPOINT
    api._transaction_handlers = {operation: api._add_current_waypoint}
    goal = ExecuteNavigationSetup.Goal(
        client_id="navigation-ui", context_id=context.context_id,
        intent=NavigationSetupIntent(
            operation=operation, map_name="plant", waypoint_name="motor_front",
        ),
    )
    handle = Mock()

    result = api._execute_in_context(handle, goal, context)

    assert result.state.state == NavigationSetupState.STATE_FAILED
    assert result.state.detail == detail
    assert result.state.context_id == context.context_id
    handle.abort.assert_called_once_with()
    handle.succeed.assert_not_called()
    assert path.read_text() == before
    assert commands.submitted == []
