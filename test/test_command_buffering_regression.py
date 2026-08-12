"""Regression tests for semantic and internal command buffering."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)


def _request(command_id):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="test-ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )


def _status(request_id, state, buffered=0, detail=""):
    return CommandExecutionStatus(
        request_id=request_id,
        state=state,
        buffered_command_count=buffered,
        detail=detail,
    )


def test_semantic_requests_remain_fifo_buffered():
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    first = _request(CommandID.STAND_UP)
    second = _request(CommandID.READY_ARM)
    third = _request(CommandID.STOW_ARM)

    controller.submit(first)
    controller.submit(second)
    controller.submit(third)

    assert dispatched == [first]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (
        second.request_id,
        third.request_id,
    )

    controller.handle_execution_status(
        _status(
            first.request_id,
            CommandControllerState.RUNNING,
            buffered=2,
            detail="BT buffered composite request",
        )
    )
    controller.handle_execution_status(
        _status(
            first.request_id,
            CommandControllerState.SUCCEEDED,
            buffered=1,
        )
    )

    assert dispatched == [first]
    assert controller.active_request_id == first.request_id

    controller.handle_execution_status(
        _status(
            first.request_id,
            CommandControllerState.SUCCEEDED,
            buffered=0,
        )
    )

    assert dispatched == [first, second]
    assert controller.active_request_id == second.request_id
    assert controller.queued_request_ids == (third.request_id,)

    controller.handle_execution_status(
        _status(second.request_id, CommandControllerState.SUCCEEDED)
    )

    assert dispatched == [first, second, third]


def test_dispatched_request_reports_transport_independent_stage():
    controller = CommandController(dispatch_request=lambda _request: None)
    request = _request(CommandID.STAND_UP)
    states = []
    controller.add_status_listener(states.append)

    controller.submit(request)

    assert states[-1].detail.startswith(
        "Dispatched stand_up to behavior tree"
    )
