"""Uncertain commands stay pending until the owner confirms stopping."""

from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import ApplicationCommandState, NavigationSetupState, ProbeSetupState

from fault_detector_spot.application.api.application_api_node import ApplicationApiNode, _OperationExecution
from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi, _NavigationExecution
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi, _ProbeMotionExecution
from fault_detector_spot.application.api.probe_point_execution_api import ProbePointExecutionApi
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import CommandControllerState
from fault_detector_spot.application.coordinators.probe_refinement_finalization_coordinator import (
    ProbeRefinementFinalizationCoordinator, _MotionWaiter,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind


@pytest.mark.parametrize("kind", ["application", "navigation", "probe"])
def test_api_action_waits_for_confirmed_stop_and_publishes_uncertainty(kind):
    goal = Mock()
    publisher = Mock()
    operation = SimpleNamespace(
        request_id="request", intent=0,
        request=SimpleNamespace(client_id="ui", context_id="context"),
    )
    harness = SimpleNamespace(
        _lock=RLock(), _state_publisher=publisher, state_publisher=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
    )
    if kind == "application":
        execution = _OperationExecution(goal, operation)
        harness._public_state = ApplicationApiNode._public_state
        harness._state_message = lambda status: ApplicationApiNode._state_message(harness, status)
        receive = lambda status: ApplicationApiNode._handle_application_status(harness, status)
        finished = execution.finished.done
        message_type = ApplicationCommandState
    elif kind == "navigation":
        execution = _NavigationExecution(goal, "request", object(), 0)
        harness._public_state = NavigationSetupApi._public_state
        harness._state = lambda snapshot, operation, code, detail, request_id: NavigationSetupState(
            state=code, detail=detail, request_id=request_id,
        )
        harness._early_states = {}
        harness.node = Mock()
        receive = lambda status: NavigationSetupApi._receive_status(harness, status)
        finished = execution.finished.is_set
        message_type = NavigationSetupState
    else:
        execution = _ProbeMotionExecution(goal, "request", object())
        harness._public_state = ProbeSetupMotionApi._public_state
        harness._motion_operation = ProbeSetupMotionApi._motion_operation
        harness.state_adapter = Mock()
        harness.state_adapter.message.side_effect = (
            lambda snapshot, operation, code, detail, **kwargs:
            ProbeSetupState(state=code, detail=detail, request_id=kwargs["request_id"])
        )
        receive = lambda status: ProbeSetupMotionApi._receive_status(harness, status)
        finished = execution.finished.is_set
        message_type = ProbeSetupState
    harness._executions = {"request": execution}

    for sequence, state in enumerate((
        CommandControllerState.UNKNOWN,
        CommandControllerState.STOP_REQUESTED,
        CommandControllerState.STOP_UNCONFIRMED,
        CommandControllerState.STOP_CONFIRMED,
    ), start=1):
        receive(SimpleNamespace(
            operation=operation, request_id="request", operation_code=0,
            state=state, detail=state.value, buffered_command_count=1,
            motion=SimpleNamespace(kind=ProbeMotionKind.MOVE_SAFE_APPROACH), snapshot=object(),
            execution_instance_id="owner-a", status_sequence=sequence,
        ))
        assert publisher.publish.call_args.args[0].state == getattr(message_type, "STATE_" + state.name)
        assert publisher.publish.call_args.args[0].execution_instance_id == "owner-a"
        assert publisher.publish.call_args.args[0].status_sequence == sequence
        assert finished() is (state is CommandControllerState.STOP_CONFIRMED)
        assert harness._executions["request"] is execution
    assert goal.publish_feedback.call_count == 4


@pytest.mark.parametrize("cancel_requested", [False, True])
def test_probe_action_confirmed_stop_uses_valid_ros_terminal_transition(cancel_requested):
    api = ProbeSetupMotionApi.__new__(ProbeSetupMotionApi)
    api._shutdown = Event()
    api._lock = RLock()
    api._executions = {}
    api._motion_request = Mock()
    api.coordinator = Mock()
    operation = SimpleNamespace(request_id="request")
    api.coordinator.prepare_motion.return_value = operation
    goal = Mock(is_cancel_requested=cancel_requested)

    def stop_confirmed(_operation):
        execution = api._executions["request"]
        execution.state = ProbeSetupState(state=ProbeSetupState.STATE_STOP_CONFIRMED)
        execution.finished.set()

    api.coordinator.submit_motion.side_effect = stop_confirmed
    result = api._execute(goal)

    assert result.state.state == ProbeSetupState.STATE_STOP_CONFIRMED
    assert goal.canceled.call_count == int(cancel_requested)
    assert goal.abort.call_count == int(not cancel_requested)
    goal.succeed.assert_not_called()
    assert api._executions == {}


def test_finalization_retraction_waits_for_confirmed_stop():
    runner = ProbeRefinementFinalizationCoordinator(Mock())
    waiter = _MotionWaiter()
    runner._waiters["motion"] = waiter
    for state in (
        CommandControllerState.UNKNOWN,
        CommandControllerState.STOP_REQUESTED,
        CommandControllerState.STOP_UNCONFIRMED,
    ):
        status = SimpleNamespace(request_id="motion", state=state)
        runner._receive_motion_status(status)
        assert waiter.status is status
        assert not waiter.event.is_set()
    stopped = SimpleNamespace(request_id="motion", state=CommandControllerState.STOP_CONFIRMED)
    runner._receive_motion_status(stopped)
    assert waiter.event.is_set()
    assert waiter.status is stopped


def test_probe_point_keeps_acquisition_correlation_until_confirmed_stop():
    api = ProbePointExecutionApi.__new__(ProbePointExecutionApi)
    api._lock = RLock()
    request = SimpleNamespace(
        request_id="request", command=SimpleNamespace(command_id=CommandID.EXECUTE_PROBE_POINT),
    )
    api._request = request
    plan = (object(),)
    api._plan = plan
    api._timer = Mock()
    api.acquisition = Mock()
    api._recording_state = "recording"
    for state in (
        CommandControllerState.UNKNOWN,
        CommandControllerState.STOP_REQUESTED,
        CommandControllerState.STOP_UNCONFIRMED,
    ):
        api._command_status(SimpleNamespace(request=request, request_id=request.request_id, state=state))
        assert api._request is request
        assert api._plan is plan
        api._timer.cancel.assert_not_called()
        api.acquisition.stop.assert_not_called()
    api._command_status(SimpleNamespace(
        request=request, request_id=request.request_id, state=CommandControllerState.STOP_CONFIRMED,
    ))
    assert api._request is None
    assert api._plan is None
    api._timer.cancel.assert_called_once_with()
    api.acquisition.stop.assert_called_once_with(cancelled=True)
