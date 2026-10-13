"""Surface measurements should be readable without opening a tooltip."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent

from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI


@pytest.mark.parametrize("intent", [
    OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE,
    OperationalIntent.INTENT_ORIENT_SENSOR_TIP_TO_SURFACE,
])
@pytest.mark.parametrize("state_id", [
    ApplicationCommandState.STATE_RUNNING,
    ApplicationCommandState.STATE_SUCCEEDED,
    ApplicationCommandState.STATE_FAILED,
])
def test_surface_action_feedback_is_visible_in_existing_status_label(intent, state_id):
    ui = SimpleNamespace(
        command_status_label=Mock(), status_label=Mock(), buffer_label=Mock(),
        _refresh_buffer_label=Mock(), _show_local_command_failure=Mock(),
    )
    state = ApplicationCommandState()
    state.intent = intent
    state.state = state_id
    state.detail = "Sensor tip distance: 261.2 mm perpendicular; axis error 0.8 deg"

    Fault_Detector_UI._process_application_state(ui, state)

    ui.status_label.setText.assert_called_once_with(state.detail)
    ui.command_status_label.setToolTip.assert_called_once_with(state.detail)
