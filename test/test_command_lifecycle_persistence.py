"""Persist real execution ownership while idle lifecycle remains nonmoving."""

from threading import Event, Thread

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.command_controller import (
    CommandController, CommandControllerState as State,
)
from test_command_controller import execution_status, make_request
from test_software_stop_controller import confirm_stop, reset_result


def test_session_is_dirty_before_dispatch_and_clean_only_after_terminal_idle():
    events = []
    controller = CommandController(
        dispatch_request=lambda request: events.append(("dispatch", request.request_id)),
        session_work_changed=lambda dirty: events.append(("dirty", dirty)),
    )
    request = make_request(CommandID.READY_ARM)
    controller.submit(request)
    assert events == [("dirty", True), ("dispatch", request.request_id)]
    controller.handle_execution_status(execution_status(request.request_id, State.UNKNOWN))
    assert events[-1] != ("dirty", False)
    controller.handle_execution_status(execution_status(request.request_id, State.SUCCEEDED, sequence=2))
    assert events[-1] == ("dirty", False)


def test_failed_ownership_persistence_prevents_motion_dispatch():
    sent = []

    def fail(_dirty):
        raise OSError("disk unavailable")

    controller = CommandController(dispatch_request=sent.append, session_work_changed=fail)
    controller.submit(make_request(CommandID.STAND_UP))
    assert sent == []
    controller.close_admission()
    assert controller.shutdown_status()[0]


def test_failed_clean_persistence_forces_new_dirty_write_before_next_dispatch():
    events, errors = [], []

    def persist(dirty):
        events.append(("persist", dirty))
        if not dirty:
            raise OSError("directory fsync failed after replacing clean marker")

    controller = CommandController(
        dispatch_request=lambda request: events.append(("dispatch", request.request_id)),
        session_work_changed=persist, listener_error_handler=errors.append,
    )
    first = make_request(CommandID.READY_ARM)
    controller.submit(first)
    controller.handle_execution_status(execution_status(first.request_id, State.SUCCEEDED))
    second = make_request(CommandID.STAND_UP)
    controller.submit(second)

    assert events == [
        ("persist", True), ("dispatch", first.request_id), ("persist", False),
        ("persist", True), ("dispatch", second.request_id),
    ]
    assert len(errors) == 1


def test_persistence_failure_cannot_suppress_a_requested_safety_stop():
    sent, errors = [], []

    def fail(_dirty):
        raise OSError("disk unavailable")

    controller = CommandController(dispatch_request=sent.append, session_work_changed=fail,
                                   listener_error_handler=errors.append)
    controller.cancel_all()
    assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL]
    assert controller.software_stop_status.latched
    assert len(errors) == 1


def test_explicit_stop_without_session_reports_once_but_reset_remains_blocked(tmp_path):
    from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore

    store = StopUnconfirmedStore(root=tmp_path)
    sent, errors = [], []
    controller = CommandController(
        dispatch_request=sent.append, session_work_changed=store.set_session_work,
        listener_error_handler=errors.append,
    )
    stop_id = controller.cancel_all("operator")
    for _ in range(20):
        controller.poll()

    assert [str(error) for error in errors] == ["No shutdown session ownership was acquired"]
    confirm_stop(controller, stop_id)
    controller.reset_software_stop("internal-client")
    assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL]
    assert controller.software_stop_status.latched
    assert not store.session_path.exists()


def test_persistence_error_reports_changes_and_recurrence_after_recovery():
    failure = ["disk unavailable"]
    attempts, errors = [], []

    def persist(dirty):
        attempts.append(dirty)
        if failure[0]:
            raise OSError(failure[0])

    controller = CommandController(
        dispatch_request=lambda _: None, session_work_changed=persist,
        listener_error_handler=errors.append,
    )
    controller.set_external_session_work(True)
    controller.poll()
    assert len(attempts) == 2  # Persistence is retried despite duplicate-log suppression.
    assert [str(error) for error in errors] == ["disk unavailable"]
    failure[0] = "permission denied"
    controller.poll()
    controller.poll()
    assert [str(error) for error in errors] == ["disk unavailable", "permission denied"]
    failure[0] = None
    controller.poll()
    failure[0] = "permission denied"
    controller.set_external_session_work(False)
    controller.poll()
    assert [str(error) for error in errors] == [
        "disk unavailable", "permission denied", "permission denied",
    ]


def test_acquisition_ownership_outlives_its_completed_start_command():
    persisted = []
    controller = CommandController(dispatch_request=lambda _: None, session_work_changed=persisted.append)
    request = make_request(CommandID.START_SENSOR_RECORDING)
    controller.submit(request)
    controller.set_external_session_work(True)
    controller.handle_execution_status(execution_status(request.request_id, State.SUCCEEDED))
    assert persisted == [True]
    controller.set_external_session_work(False)
    assert persisted == [True, False]


def test_confirmed_operator_stop_remains_dirty_until_explicit_reset():
    persisted = []
    controller = CommandController(dispatch_request=lambda _: None, session_work_changed=persisted.append)
    confirm_stop(controller, controller.cancel_all())
    assert persisted == [True]
    request_id = controller.reset_software_stop()
    controller.handle_execution_status(reset_result(request_id))
    assert persisted == [True, False]


def test_idle_shutdown_closes_internal_admission_and_cancels_queue_without_stop():
    sent, persisted = [], []
    controller = CommandController(dispatch_request=sent.append, dispatch_ready=lambda: False,
                                   session_work_changed=persisted.append)
    controller.submit(make_request(CommandID.STAND_UP))
    controller.close_admission()
    controller.poll()
    assert controller.shutdown_status()[0]
    assert controller.queued_request_ids == ()
    assert sent == persisted == []
    assert not controller.software_stop_status.latched
    with pytest.raises(RuntimeError, match="closed command admission"):
        controller.submit(make_request(CommandID.STAND_UP))


def test_shutdown_retains_actual_motion_and_rejects_late_cancellation():
    sent, persisted = [], []
    controller = CommandController(dispatch_request=sent.append, session_work_changed=persisted.append)
    request = make_request(CommandID.STAND_UP)
    controller.submit(request)
    controller.close_admission()
    assert not controller.shutdown_status()[0]
    with pytest.raises(RuntimeError, match="closed command admission"):
        controller.cancel_all()
    with pytest.raises(RuntimeError, match="closed command admission"):
        controller.cancel(request.request_id)
    controller.poll()
    assert sent == [request]
    assert controller.active_request_id == request.request_id
    assert persisted == [True]


def test_passive_startup_fence_does_not_write_without_session_or_allow_reset(tmp_path):
    from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore
    from fault_detector_spot.application.controllers.command_controller import (
        CommandExecutionUnresolved, SoftwareStopLatched,
    )

    store = StopUnconfirmedStore(root=tmp_path)
    sent, errors = [], []
    controller = CommandController(
        sent.append, session_work_changed=store.set_session_work,
        listener_error_handler=errors.append,
    )
    controller.restore_shutdown_uncertainty("Another application API owns this session")
    controller.observe_owner_software_stop(True, True)
    for _ in range(20):
        controller.poll()
    with pytest.raises(SoftwareStopLatched):
        controller.submit(make_request(CommandID.STAND_UP))
    with pytest.raises(CommandExecutionUnresolved):
        controller.reset_software_stop()
    controller.close_admission()
    assert controller.shutdown_status()[0]
    assert sent == errors == []
    assert not store.session_path.exists()


def test_passive_remote_stop_does_not_dirty_local_session_or_wait_on_shutdown():
    sent, persisted = [], []
    controller = CommandController(sent.append, session_work_changed=persisted.append)
    controller.observe_owner_software_stop(True, False)
    controller.close_admission()
    controller.poll()
    assert controller.shutdown_status()[0]
    assert controller.software_stop_status.latched
    assert sent == persisted == []


def test_shutdown_does_not_dispatch_previously_requested_stop_when_transport_appears():
    sent, ready = [], [False]
    controller = CommandController(sent.append, dispatch_ready=lambda: ready[0])
    controller.cancel_all()
    controller.close_admission()
    ready[0] = True
    controller.poll()
    assert sent == []
    assert not controller.shutdown_status()[0]


def test_shutdown_fences_command_preparation_before_waiting_for_its_lock():
    entered, release, closing = Event(), Event(), Event()
    sent, errors = [], []
    controller = CommandController(dispatch_request=sent.append)

    def prepare(request):
        entered.set()
        assert release.wait(3)
        return request

    def submit():
        try:
            controller.submit(make_request(CommandID.STAND_UP))
        except RuntimeError as error:
            errors.append(error)

    controller.add_request_preparer(prepare)
    worker = Thread(target=submit)
    worker.start()
    assert entered.wait(2)
    closer = Thread(target=lambda: (controller.close_admission(), closing.set()))
    closer.start()
    try:
        # Closing sets the admission fence under the emergency lock, before it
        # waits for the ordinary lock held by the command preparer.
        for _ in range(100):
            with controller._emergency_lock:
                if controller._admission_closed:
                    break
            Event().wait(.001)
        assert controller._admission_closed
    finally:
        release.set()
        worker.join(3)
        closer.join(3)
    assert closing.is_set()
    assert len(errors) == 1
    assert sent == []
    assert controller.shutdown_status()[0]
