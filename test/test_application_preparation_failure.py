"""Preparation errors must terminate accepted actions without dispatching."""

import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.action import ExecuteOperation
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent

from fault_detector_spot.application.api.application_api_node import (
    ApplicationApiNode,
)
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)


class ApiHarness(SimpleNamespace):
    _execute_operation = ApplicationApiNode._execute_operation
    _abort_preparation = ApplicationApiNode._abort_preparation


@pytest.mark.parametrize("error", [
    ValueError("Newest base-tag observation is stale; newest_age=1.513 s"),
    RuntimeError("Sensor attachment state is unavailable"),
    FileNotFoundError("Object definition does not exist"),
])
@pytest.mark.parametrize("intent_id", [
    OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH,
    OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH,
])
def test_saved_probe_preparation_failure_aborts_with_correlated_state(
    error, intent_id,
):
    commands = Mock()
    controller = ApplicationController(commands)
    controller.probe_setup_coordinator = Mock()
    controller.probe_setup_coordinator.saved_probe_command.side_effect = error
    stamp = Time(sec=12)
    node = ApiHarness(
        application_controller=controller,
        _executions={},
        _state_publisher=Mock(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: stamp),
        ),
    )
    goal = ExecuteOperation.Goal()
    goal.client_id = "probe-ui"
    goal.context_id = " saved-probe-context "
    goal.intent.intent = intent_id
    goal.intent.object_id = "motor"
    goal.intent.routine_id = "scan"
    goal.intent.probe_point_id = "point1"
    handle = Mock(request=goal)

    result = asyncio.run(node._execute_operation(handle))

    assert isinstance(result, ExecuteOperation.Result)
    assert result.state.state == ApplicationCommandState.STATE_FAILED
    assert result.state.detail == str(error)
    assert result.state.request_id
    assert result.state.client_id == "probe-ui"
    assert result.state.context_id == "saved-probe-context"
    assert result.state.intent == intent_id
    assert result.state.header.stamp == stamp
    handle.abort.assert_called_once_with()
    handle.succeed.assert_not_called()
    handle.canceled.assert_not_called()
    node._state_publisher.publish.assert_called_once_with(result.state)
    commands.submit.assert_not_called()
    assert controller._operations == {}
    assert node._executions == {}
