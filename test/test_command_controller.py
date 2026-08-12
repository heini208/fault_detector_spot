"""Tests for transport-independent semantic command execution."""

import pytest

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
    DuplicateCommandRequest,
)


def make_request(command_id, client_id="operator_ui"):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id=client_id,
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )


def execution_status(request_id, state, buffered_count=0, detail=""):
    return CommandExecutionStatus(
        request_id=request_id,
        state=state,
        detail=detail,
        buffered_command_count=buffered_count,
    )


def test_controller_dispatches_only_one_semantic_request_at_a_time():
    dispatched = []
    accepted = []
    states = []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.add_accepted_listener(accepted.append)
    controller.add_status_listener(states.append)
    first = make_request(CommandID.STAND_UP)
    second = make_request(CommandID.STOW_ARM)

    controller.submit(first)
    controller.submit(second)

    assert accepted == [first, second]
    assert dispatched == [first]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (second.request_id,)
    assert isinstance(controller._active.command, SemanticCommand)

    controller.handle_execution_status(execution_status(
        first.request_id,
        CommandControllerState.SUCCEEDED,
    ))

    assert dispatched == [first, second]
    assert controller.active_request_id == second.request_id
    assert states[-2].state is CommandControllerState.SUCCEEDED
    assert states[-1].state is CommandControllerState.DISPATCHED


def test_internal_success_does_not_release_the_next_request():
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    first = make_request(CommandID.MOVE_ARM_TO_TAG_AND_WAIT)
    second = make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)

    handled = controller.handle_execution_status(execution_status(
        first.request_id,
        CommandControllerState.SUCCEEDED,
        buffered_count=1,
    ))

    assert handled is True
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (second.request_id,)
    assert dispatched == [first]


def test_duplicate_request_identity_is_rejected():
    controller = CommandController(dispatch_request=lambda _request: None)
    request = make_request(CommandID.STAND_UP)
    controller.submit(request)

    with pytest.raises(DuplicateCommandRequest):
        controller.submit(request)


def test_emergency_stop_clears_queue_and_bypasses_active_request():
    dispatched = []
    states = []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.add_status_listener(states.append)
    first = make_request(CommandID.MOVE_TO_WAYPOINT)
    second = make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)

    emergency_request_id = controller.cancel_all()

    assert [request.request_id for request in dispatched] == [
        first.request_id,
        emergency_request_id,
    ]
    assert controller.active_request_id == emergency_request_id
    assert controller.queued_request_ids == ()
    cancelled = {
        status.request_id
        for status in states
        if status.state is CommandControllerState.CANCELLED
    }
    assert cancelled == {first.request_id, second.request_id}
    assert dispatched[-1].command.command_id is CommandID.EMERGENCY_CANCEL


def test_dispatch_reports_transport_independent_stage():
    states = []
    controller = CommandController(dispatch_request=lambda _request: None)
    controller.add_status_listener(states.append)

    controller.submit(make_request(CommandID.READY_ARM))

    assert states[-1].state is CommandControllerState.DISPATCHED
    assert states[-1].detail.startswith(
        "Dispatched ready_arm to behavior tree"
    )


def test_controller_rejects_non_semantic_payload():
    request = CommandRequest.create(
        command=object(),
        client_id="operator-ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.EXCLUDE,
    )
    controller = CommandController(dispatch_request=lambda _request: None)

    with pytest.raises(TypeError, match="SemanticCommand"):
        controller.submit(request)


def test_unacknowledged_dispatch_fails_without_ros_clock_or_messages():
    now = {"value": 10.0}
    states = []
    controller = CommandController(
        dispatch_request=lambda _request: None,
        monotonic_clock=lambda: now["value"],
        ack_timeout_sec=3.0,
    )
    controller.add_status_listener(states.append)
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)

    now["value"] = 13.1
    controller.poll()

    assert controller.active_request_id == ""
    assert states[-1].state is CommandControllerState.FAILED
    assert "did not acknowledge" in states[-1].detail
