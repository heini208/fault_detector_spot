"""Offline contracts for the application-owned software stop latch."""

from dataclasses import replace
from threading import Event, Thread

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import RecordingPolicy
from fault_detector_spot.application.controllers.command_controller import (
    CommandController, CommandControllerState as State, CommandExecutionUnresolved,
    SoftwareStopLatched,
)
from test_command_controller import execution_status, make_request


def confirm_stop(controller, stop_id):
    controller.poll()
    assert controller.handle_execution_status(execution_status(stop_id, State.STOP_CONFIRMED))


def reset_result(request_id, **kwargs):
    return replace(execution_status(request_id, State.SUCCEEDED),
                   software_stop_confirmed=True, **kwargs)


@pytest.mark.parametrize("command", [CommandID.STAND_UP, CommandID.STOW_ARM,
                                    CommandID.RECOVER_ARM, CommandID.START_SENSOR_RECORDING])
def test_confirmed_stop_remains_latched_and_rejects_new_normal_admission(command):
    sent = []
    controller = CommandController(sent.append)
    stop_id = controller.cancel_all()
    confirm_stop(controller, stop_id)
    assert controller.software_stop_status.stop_confirmed
    assert controller.cancel_all() == stop_id
    with pytest.raises(SoftwareStopLatched):
        controller.submit(make_request(command))
    assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL]


def test_acquisition_proof_is_required_even_when_start_command_already_finished():
    sent, aborts, resets, local = [], [], [], [False]
    controller = CommandController(sent.append)
    controller.configure_stop_participant(aborts.append, lambda key: (local[0], "sensor still active"),
                                          reset=lambda: resets.append(True))
    stop_id = controller.cancel_all()
    assert aborts == [stop_id]
    confirm_stop(controller, stop_id)
    assert not controller.software_stop_status.stop_confirmed
    with pytest.raises(CommandExecutionUnresolved):
        controller.reset_software_stop()
    with pytest.raises(RuntimeError):
        controller.run_if_idle(lambda: pytest.fail("Unconfirmed stop admitted setup mutation"))
    local[0] = True
    controller.poll()
    assert controller.software_stop_status.stop_confirmed
    reset_id = controller.reset_software_stop()
    assert resets == []
    assert controller.software_stop_status.latched
    assert controller.handle_execution_status(reset_result(reset_id))
    assert resets == [True]
    assert not controller.software_stop_status.latched


def test_reset_is_nonmoving_excluded_and_does_not_replay_queued_work():
    sent, prepared = [], []
    controller = CommandController(sent.append)
    controller.add_request_preparer(lambda request: (prepared.append(request), request)[1])
    active, queued = make_request(CommandID.STAND_UP), make_request(CommandID.CLOSE_GRIPPER)
    controller.submit(active)
    controller.submit(queued)
    stop_id = controller.cancel_all()
    controller.poll()
    controller.handle_execution_status(execution_status(active.request_id, State.STOP_CONFIRMED))
    confirm_stop(controller, stop_id)
    assert controller.queued_request_ids == ()
    reset_id = controller.reset_software_stop()
    assert controller.reset_software_stop() == reset_id
    assert sent[-1].command.command_id is CommandID.RESET_SOFTWARE_STOP
    assert sent[-1].recording_policy is RecordingPolicy.EXCLUDE
    assert prepared == [active, queued]
    controller.handle_execution_status(reset_result(reset_id))
    controller.poll()
    assert [item.command.command_id for item in sent] == [
        CommandID.STAND_UP, CommandID.EMERGENCY_CANCEL, CommandID.RESET_SOFTWARE_STOP,
    ]
    assert controller.software_stop_status.request_id == stop_id
    fresh = make_request(CommandID.RECOVER_ARM)
    controller.submit(fresh)
    assert sent[-1] == fresh
    assert queued not in sent


@pytest.mark.parametrize("state,latched,confirmed", [
    (State.FAILED, True, True), (State.SUCCEEDED, True, True), (State.SUCCEEDED, False, False),
])
def test_incomplete_or_failed_reset_keeps_motion_blocked(state, latched, confirmed):
    controller = CommandController(lambda request: None)
    confirm_stop(controller, controller.cancel_all())
    reset_id = controller.reset_software_stop()
    controller.handle_execution_status(replace(execution_status(reset_id, state),
        software_stop_latched=latched, software_stop_confirmed=confirmed))
    assert controller.software_stop_status.latched
    with pytest.raises(SoftwareStopLatched):
        controller.submit(make_request(CommandID.READY_ARM))


def test_uncertain_reset_keeps_latch_and_correlates_late_owner_confirmation():
    sent, now = [], [0.0]
    controller = CommandController(sent.append, monotonic_clock=lambda: now[0])
    confirm_stop(controller, controller.cancel_all())
    reset_id = controller.reset_software_stop()
    now[0] = 3.1
    controller.poll()
    assert controller.active_request_id == reset_id
    assert controller.reset_software_stop() == reset_id
    assert controller.software_stop_status.latched
    assert len(sent) == 2
    controller.handle_execution_status(reset_result(reset_id))
    assert not controller.software_stop_status.latched
    assert len(sent) == 2


def test_cancelling_undispatched_reset_does_not_strand_the_latch():
    sent = []
    controller = CommandController(sent.append)
    confirm_stop(controller, controller.cancel_all())
    controller.configure_dispatch(sent.append, reset_ready=lambda: False)
    first = controller.reset_software_stop()
    controller.cancel(first)
    second = controller.reset_software_stop()
    assert second != first
    assert controller.queued_request_ids == (second,)
    controller.configure_dispatch(sent.append, reset_ready=lambda: True)
    assert sent[-1].request_id == second
    assert controller.software_stop_status.latched


def test_new_stop_during_reset_vetoes_old_reset_and_requires_new_confirmation():
    sent = []
    controller = CommandController(sent.append)
    first_stop = controller.cancel_all()
    confirm_stop(controller, first_stop)
    reset_id = controller.reset_software_stop()
    second_stop = controller.cancel_all()
    assert second_stop != first_stop
    assert reset_id in sent[-1].stop_request_ids
    controller.poll()
    assert controller.handle_execution_status(reset_result(reset_id))
    assert controller.software_stop_status.latched
    assert not controller.software_stop_status.stop_confirmed
    confirm_stop(controller, second_stop)
    assert not controller.software_stop_status.stop_confirmed
    controller.handle_execution_status(execution_status(reset_id, State.STOP_CONFIRMED, sequence=2))
    assert controller.software_stop_status.latched
    assert controller.software_stop_status.stop_confirmed
    assert controller.software_stop_status.request_id == second_stop
    with pytest.raises(SoftwareStopLatched):
        controller.submit(make_request(CommandID.STAND_UP))


def test_stop_between_reset_validation_and_admission_vetoes_reset():
    entered, release = Event(), Event()

    class PauseAfterUnlock:
        def __init__(self, lock):
            self.lock, self.pause = lock, True

        def __enter__(self):
            return self.lock.__enter__()

        def __exit__(self, *args):
            result = self.lock.__exit__(*args)
            if self.pause:
                self.pause = False
                entered.set()
                assert release.wait(3)
            return result

    class PausingController(CommandController):
        pause_reset = False

        def _refresh_stop_confirmation_locked(self):
            super()._refresh_stop_confirmation_locked()
            if self.pause_reset:
                self.pause_reset = False
                self._emergency_lock = PauseAfterUnlock(self._emergency_lock)

    sent, rejected = [], []
    controller = PausingController(sent.append)
    original = controller.cancel_all()
    confirm_stop(controller, original)
    controller.pause_reset = True

    def reset():
        try:
            controller.reset_software_stop()
        except SoftwareStopLatched as exception:
            rejected.append(exception)

    worker = Thread(target=reset, daemon=True)
    worker.start()
    try:
        assert entered.wait(2)
        newer = controller.cancel_all()
        assert newer != original
    finally:
        release.set()
        worker.join(3)
    assert len(rejected) == 1
    assert controller.software_stop_status.latched
    assert not controller.software_stop_status.stop_confirmed
    assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL] * 2


def test_stop_confirmation_listener_cannot_block_immediate_repeated_stop():
    entered, release, completed = Event(), Event(), Event()
    controller = CommandController(lambda request: None)
    stop_id = controller.cancel_all()
    controller.poll()

    def listener(status):
        if status.state is State.STOP_CONFIRMED:
            entered.set()
            assert release.wait(3)

    controller.add_status_listener(listener)
    feedback = Thread(target=lambda: controller.handle_execution_status(
        execution_status(stop_id, State.STOP_CONFIRMED)), daemon=True)
    feedback.start()
    try:
        assert entered.wait(2)
        stopper = Thread(target=lambda: (controller.cancel_all(), completed.set()), daemon=True)
        stopper.start()
        assert completed.wait(2), "Ordinary stop listener blocked priority stop admission"
    finally:
        release.set()
        feedback.join(3)
    stopper.join(3)
    assert controller.software_stop_status.latched


def test_owner_restart_invalidates_confirmed_stop_without_replay_or_auto_reset():
    sent = []
    controller = CommandController(sent.append)
    confirm_stop(controller, controller.cancel_all())
    controller.execution_owner_changed("test-execution-owner", "replacement-owner")
    assert controller.software_stop_status.latched
    assert not controller.software_stop_status.stop_confirmed
    with pytest.raises(CommandExecutionUnresolved):
        controller.reset_software_stop()
    assert len(sent) == 1


@pytest.mark.parametrize("loss", ["contradiction", "heartbeat", "expired_receipt", "restart"])
def test_local_stop_finishing_cannot_restore_revoked_owner_stop_proof(loss):
    now, local = [0.0], [False]
    controller = CommandController(lambda request: None, monotonic_clock=lambda: now[0])
    controller.configure_stop_participant(lambda key: None, lambda key: (local[0], "sensor stopping"))
    stop_id = controller.cancel_all()
    confirm_stop(controller, stop_id)
    assert not controller.software_stop_status.stop_confirmed
    if loss == "contradiction":
        controller.handle_execution_status(execution_status(stop_id, State.STOP_UNCONFIRMED, sequence=2))
    elif loss == "heartbeat":
        now[0] = 5.1
    elif loss == "expired_receipt":
        now[0] = 6.0
        controller.handle_execution_status(execution_status(stop_id, State.STOP_CONFIRMED, sequence=2),
                                           receipt_monotonic=0.5)
    else:
        controller.execution_owner_changed("test-execution-owner", "replacement")
    local[0] = True
    controller.poll()
    assert not controller.software_stop_status.stop_confirmed
    with pytest.raises(CommandExecutionUnresolved):
        controller.reset_software_stop()
    assert controller.active_request_id == stop_id


@pytest.mark.parametrize("loss", ["owner_query", "heartbeat"])
def test_completed_stop_remains_reconciled_and_revoked_proof_blocks_reset(loss):
    now, queries = [0.0], []
    controller = CommandController(lambda request: None, monotonic_clock=lambda: now[0])
    controller.configure_reconciliation(queries.append)
    stop_id = controller.cancel_all()
    confirm_stop(controller, stop_id)
    if loss == "owner_query":
        controller.observe_owner_software_stop(True, False)
    else:
        now[0] = 5.1
        controller.poll()
        assert queries == [stop_id]
    assert not controller.software_stop_status.stop_confirmed
    with pytest.raises(CommandExecutionUnresolved):
        controller.reset_software_stop()
    assert controller.handle_execution_status(replace(
        execution_status(stop_id, State.STOP_CONFIRMED, sequence=2),
        software_stop_latched=True, software_stop_confirmed=True))
    assert controller.software_stop_status.stop_confirmed
    assert controller.software_stop_status.latched


def test_latch_snapshots_are_ordered_and_heartbeat_preserves_reset_identity():
    now, snapshots = [0.0], []
    controller = CommandController(lambda request: None, monotonic_clock=lambda: now[0])
    controller.add_software_stop_listener(snapshots.append)
    assert snapshots[-1].status_sequence > 0
    stop_id = controller.cancel_all()
    confirm_stop(controller, stop_id)
    reset_id = controller.reset_software_stop()
    controller.handle_execution_status(reset_result(reset_id))
    now[0] = 2.0
    controller.poll()
    assert not snapshots[-1].latched
    assert snapshots[-1].request_id == stop_id
    sequences = [item.status_sequence for item in snapshots]
    assert sequences == sorted(set(sequences))
