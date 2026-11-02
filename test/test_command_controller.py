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
        execution_idle=state in {CommandControllerState.SUCCEEDED, CommandControllerState.FAILED,
                                 CommandControllerState.CANCELLED, CommandControllerState.STOP_CONFIRMED},
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
    controller.poll()

    assert [request.request_id for request in dispatched] == [
        first.request_id,
        emergency_request_id,
    ]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == ()
    cancelled = {
        status.request_id
        for status in states
        if status.state is CommandControllerState.CANCELLED
    }
    assert cancelled == {second.request_id}
    assert states[-2].state is CommandControllerState.STOP_REQUESTED
    assert dispatched[-1].stop_request_ids == (first.request_id,)
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


def test_unacknowledged_dispatch_retains_ownership_without_ros_clock_or_messages():
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

    assert controller.active_request_id == request.request_id
    assert states[-1].state is CommandControllerState.UNKNOWN
    assert "did not acknowledge" in states[-1].detail


def test_lost_ack_queries_owner_without_resending_and_accepts_late_feedback():
    now = [0.0]
    dispatched, queried, states = [], [], []
    controller = CommandController(dispatched.append, monotonic_clock=lambda: now[0])
    controller.configure_reconciliation(queried.append)
    controller.add_status_listener(states.append)
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)
    for instant in (3.1, 3.2, 6.2):
        now[0] = instant
        controller.poll()
    assert queried == [first.request_id, first.request_id]
    assert dispatched == [first]
    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (second.request_id,)
    with pytest.raises(RuntimeError):
        controller.run_if_idle(lambda: pytest.fail("Unresolved lane admitted setup mutation"))
    assert controller.handle_execution_status(execution_status(first.request_id, CommandControllerState.RUNNING))
    assert controller.active_request_id == first.request_id
    assert controller.handle_execution_status(execution_status(first.request_id, CommandControllerState.SUCCEEDED))
    assert dispatched == [first, second]
    assert not controller.handle_execution_status(execution_status(first.request_id, CommandControllerState.RUNNING))


@pytest.mark.parametrize("state", [CommandControllerState.SUCCEEDED, CommandControllerState.FAILED,
                                  CommandControllerState.CANCELLED, CommandControllerState.STOP_CONFIRMED])
def test_terminal_without_nonexecuting_proof_retains_lane_and_disables_continuation(state):
    dispatched, states = [], []
    controller = CommandController(dispatched.append)
    controller.add_status_listener(states.append)
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)
    assert controller.handle_execution_status(CommandExecutionStatus(
        first.request_id, state, retry_safe=True, continuation_safe=True,
    ))
    assert controller.active_request_id == first.request_id
    assert dispatched == [first]
    assert states[-1].state is CommandControllerState.UNKNOWN
    assert not states[-1].retry_safe and not states[-1].continuation_safe
    assert states[-1].execution_idle is False


def test_submission_exception_after_possible_delivery_does_not_advance_or_resend():
    dispatched, states = [], []
    def send(request):
        dispatched.append(request)
        raise RuntimeError("publish outcome unavailable")
    controller = CommandController(send)
    controller.add_status_listener(states.append)
    first, second = make_request(CommandID.READY_ARM), make_request(CommandID.STOW_ARM)
    controller.submit(first)
    controller.submit(second)
    controller.poll()
    assert dispatched == [first]
    assert controller.active_request_id == first.request_id
    assert any(status.state is CommandControllerState.UNKNOWN for status in states)
    controller.configure_dispatch(dispatched.append)
    assert dispatched == [first]
    controller.handle_execution_status(execution_status(first.request_id, CommandControllerState.SUCCEEDED))
    assert dispatched == [first, second]


def test_running_request_queries_for_lost_terminal_feedback():
    now, queried = [0.0], []
    controller = CommandController(lambda _: None, monotonic_clock=lambda: now[0])
    controller.configure_reconciliation(queried.append)
    request = make_request(CommandID.STAND_UP)
    controller.submit(request)
    controller.handle_execution_status(execution_status(request.request_id, CommandControllerState.RUNNING))
    now[0] = 4.0
    controller.poll()
    assert queried == [request.request_id]
    controller.handle_execution_status(CommandExecutionStatus(request.request_id, CommandControllerState.UNKNOWN))
    assert controller.active_request_id == request.request_id


def test_synchronous_ack_is_not_overwritten_by_dispatched_notification():
    states = []
    controller = CommandController()
    controller.configure_dispatch(lambda request: controller.handle_execution_status(
        execution_status(request.request_id, CommandControllerState.RUNNING)))
    controller.add_status_listener(states.append)
    controller.submit(make_request(CommandID.STAND_UP))
    assert [status.state for status in states] == [CommandControllerState.QUEUED, CommandControllerState.RUNNING]


@pytest.mark.parametrize("original_first", [True, False])
def test_stop_requires_both_original_and_stop_owner_confirmation(original_first):
    dispatched, states = [], []
    controller = CommandController(dispatched.append)
    controller.add_status_listener(states.append)
    first = make_request(CommandID.READY_ARM)
    controller.submit(first)
    stop_id = controller.cancel_all()
    controller.poll()
    second = make_request(CommandID.STOW_ARM)
    controller.submit(second)
    # A start result or cancellation acceptance is not physical stop proof.
    controller.handle_execution_status(execution_status(first.request_id, CommandControllerState.SUCCEEDED))
    controller.handle_execution_status(execution_status(stop_id, CommandControllerState.SUCCEEDED))
    assert len(dispatched) == 2
    assert controller.active_request_id == first.request_id
    ids = [first.request_id, stop_id] if original_first else [stop_id, first.request_id]
    controller.handle_execution_status(execution_status(ids[0], CommandControllerState.STOP_CONFIRMED))
    assert len(dispatched) == 2
    controller.handle_execution_status(execution_status(ids[1], CommandControllerState.STOP_CONFIRMED))
    assert dispatched[-1] == second
    assert all(status.execution_idle for status in states if status.state is CommandControllerState.STOP_CONFIRMED)


def test_failed_stop_and_retry_keep_original_identity_and_fence_previous_attempt():
    now, dispatched = [0.0], []
    controller = CommandController(dispatched.append, monotonic_clock=lambda: now[0])
    original = make_request(CommandID.READY_ARM)
    controller.submit(original)
    stop_id = controller.cancel_all()
    controller.poll()
    controller.handle_execution_status(CommandExecutionStatus(stop_id, CommandControllerState.FAILED))
    now[0] = 4.0
    controller.poll()
    assert controller.active_request_id == original.request_id
    retry_id = controller.cancel_all()
    controller.poll()
    assert set(dispatched[-1].stop_request_ids) == {original.request_id, stop_id}
    for request_id in (original.request_id, stop_id, retry_id):
        controller.handle_execution_status(execution_status(request_id, CommandControllerState.STOP_CONFIRMED))
    assert controller.active_request_id == ""


def test_stop_confirmation_before_bookkeeping_is_buffered_and_reconciled():
    controller = CommandController()
    def dispatch(request):
        if request.command.command_id is CommandID.EMERGENCY_CANCEL:
            controller.handle_execution_status(execution_status(request.request_id, CommandControllerState.STOP_CONFIRMED))
    controller.configure_dispatch(dispatch)
    stop_id = controller.cancel_all()
    controller.poll()
    assert controller.active_request_id == ""
    assert controller.knows_request(stop_id)
