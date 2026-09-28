"""Offline regression tests for the emergency-only dispatch bypass."""

from threading import Event, Thread

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    DuplicateCommandRequest,
)
from test_command_controller import make_request, execution_status


@pytest.mark.parametrize("blocker", ["preparer", "accepted", "status", "idle"])
def test_emergency_dispatch_does_not_wait_for_setup_work(blocker):
    entered, release = Event(), Event()
    dispatched, states = [], []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.add_status_listener(states.append)
    request = make_request(CommandID.READY_ARM)

    def block(value=None):
        entered.set()
        assert release.wait(3), "Test did not release blocked setup work"
        return value

    if blocker == "preparer":
        controller.add_request_preparer(block)
    elif blocker == "accepted":
        controller.add_accepted_listener(block)
    elif blocker == "status":
        controller.add_status_listener(block)
    work = (lambda: controller.run_if_idle(block)) if blocker == "idle" else (
        lambda: controller.submit(request)
    )
    worker = Thread(target=work, daemon=True)
    worker.start()
    try:
        assert entered.wait(2)
        stopped = Event()
        stopper = Thread(target=lambda: (controller.cancel_all(), stopped.set()), daemon=True)
        stopper.start()
        assert stopped.wait(2), "Emergency waited for the setup lock/callback"
        assert [item.command.command_id for item in dispatched] == [CommandID.EMERGENCY_CANCEL]
    finally:
        release.set()
        worker.join(3)
    stopper.join(3)
    controller.poll()
    assert [item.command.command_id for item in dispatched] == [CommandID.EMERGENCY_CANCEL]
    if blocker != "idle":
        assert any(item.request_id == request.request_id and
                   item.state is CommandControllerState.CANCELLED for item in states)


def test_stop_is_published_before_cancellation_listeners_run():
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.submit(make_request(CommandID.READY_ARM))
    observed = []

    def listener(status):
        if status.state is CommandControllerState.CANCELLED:
            observed.append(dispatched[-1].command.command_id)

    controller.add_status_listener(listener)
    controller.cancel_all()
    assert observed == []
    controller.poll()
    assert observed == [CommandID.EMERGENCY_CANCEL]


def test_early_emergency_feedback_is_preserved_until_reconciliation():
    controller = CommandController()
    states = []
    controller.add_status_listener(states.append)

    def dispatch(request):
        assert controller.handle_execution_status(execution_status(
            request.request_id, CommandControllerState.SUCCEEDED,
        ))

    controller.configure_dispatch(dispatch)
    request_id = controller.cancel_all()
    controller.poll()
    assert controller.active_request_id == ""
    assert states[-1].request_id == request_id
    assert states[-1].state is CommandControllerState.SUCCEEDED


def test_repeated_emergencies_dispatch_before_bookkeeping():
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    first = controller.cancel_all()
    second = controller.cancel_all()
    assert [request.request_id for request in dispatched] == [first, second]
    controller.poll()
    assert controller.active_request_id == second


def test_duplicate_emergency_is_not_sent_twice():
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    request = make_request(CommandID.EMERGENCY_CANCEL)
    controller.submit(request)
    with pytest.raises(DuplicateCommandRequest):
        controller.submit(request)
    assert len(dispatched) == 1


def test_emergency_transport_failure_is_reported_without_releasing_old_work():
    states = []
    controller = CommandController(dispatch_ready=lambda: False)
    controller.submit(make_request(CommandID.READY_ARM))
    controller.add_status_listener(states.append)

    def fail(_request):
        raise RuntimeError("Transport unavailable")

    controller.configure_dispatch(fail, lambda: False)
    controller._dispatch_ready = lambda: True
    stop_id = controller.cancel_all()
    controller.poll()
    assert controller.queued_request_ids == ()
    assert controller.active_request_id == ""
    assert states[-1].request_id == stop_id
    assert states[-1].state is CommandControllerState.FAILED
    assert "Transport unavailable" in states[-1].detail


def test_second_stop_bypasses_blocked_cancellation_bookkeeping():
    entered, release, stopped = Event(), Event(), Event()
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.submit(make_request(CommandID.READY_ARM))

    def listener(status):
        if status.state is CommandControllerState.CANCELLED:
            entered.set()
            assert release.wait(3)

    controller.add_status_listener(listener)
    controller.cancel_all()
    bookkeeping = Thread(target=controller.poll, daemon=True)
    bookkeeping.start()
    try:
        assert entered.wait(2)
        stopper = Thread(
            target=lambda: (controller.cancel_all(), stopped.set()),
            daemon=True,
        )
        stopper.start()
        assert stopped.wait(2)
        assert [request.command.command_id for request in dispatched] == [
            CommandID.READY_ARM,
            CommandID.EMERGENCY_CANCEL,
            CommandID.EMERGENCY_CANCEL,
        ]
    finally:
        release.set()
        bookkeeping.join(3)
    stopper.join(3)
    controller.poll()
    assert controller.active_request_id == dispatched[-1].request_id
