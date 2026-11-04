"""Probe cancellation and physical admission remain live during feedback."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event, current_thread

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin, CommandRequest, RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.controllers.command_controller import (
    CommandController, CommandControllerState, CommandExecutionStatus,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import ProbeSetupCoordinator
from fault_detector_spot.application.coordinators.setup_coordinator import SetupCoordinator
from fault_detector_spot.inspection.repository.multi_reference_view_repository import MultiReferenceViewRepository
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind, ProbeMotionRequest

from test_probe_setup_coordinator import (
    FakeGeometry, FakeMotionCommandFactory, FakeMotionStateSource,
    FakeSensorAttachmentController, create_selected_routine,
)


class FeedbackRace:
    """Pause real feedback until its competing workflow reaches command access.

    The real command lock still serializes execution; bounded acquisition turns
    a lock inversion into a test failure instead of stranding pytest workers.
    """

    def __init__(self, commands):
        self.lock = commands._lock
        commands._lock = self
        self.entered = Event()
        self.contending = Event()

    def __enter__(self):
        if current_thread().name.startswith("workflow"):
            self.contending.set()
        if not self.lock.acquire(timeout=2):
            raise TimeoutError("Workflow and command feedback deadlocked")
        return self

    def __exit__(self, *_exception):
        self.lock.release()

    def pause_feedback(self, _status):
        if current_thread().name.startswith("feedback"):
            self.entered.set()
            assert self.contending.wait(4), "Workflow never reached command access"

    def run(self, feedback, workflow, workflow_ready=None):
        with (
            ThreadPoolExecutor(max_workers=1, thread_name_prefix="feedback") as delivery,
            ThreadPoolExecutor(max_workers=1, thread_name_prefix="workflow") as mutation,
        ):
            try:
                if workflow_ready is not None:
                    result = mutation.submit(workflow)
                    assert workflow_ready.wait(4), "Workflow did not start"
                    status = delivery.submit(feedback)
                else:
                    status = delivery.submit(feedback)
                    assert self.entered.wait(4), "Command feedback did not start"
                    result = mutation.submit(workflow)
                return result.result(timeout=5), status.result(timeout=5)
            finally:
                self.contending.set()


def live_coordinator(tmp_path):
    dispatched, errors = [], []
    commands = CommandController(
        dispatch_request=dispatched.append, listener_error_handler=errors.append,
    )
    race = FeedbackRace(commands)
    commands.add_status_listener(race.pause_feedback)
    setup = SetupCoordinator(commands)
    attachment = FakeSensorAttachmentController()
    probe = ProbeSetupCoordinator(
        setup, MultiReferenceViewRepository(tmp_path / "objects"), attachment,
        geometry=FakeGeometry(), motion_state_source=FakeMotionStateSource(),
        motion_command_factory=FakeMotionCommandFactory(),
    )
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    return probe, commands, setup, attachment, state, race, dispatched, errors


def execution_status(request_id, state, sequence=1):
    return CommandExecutionStatus(
        request_id=request_id, state=state, execution_instance_id="robot-owner",
        status_sequence=sequence,
        execution_idle=state in {
            CommandControllerState.SUCCEEDED, CommandControllerState.STOP_CONFIRMED,
        },
    )


@pytest.mark.parametrize("admission", ["begin", "prepare_motion"])
def test_external_command_racing_refinement_admission_rejects_without_deadlock(tmp_path, admission):
    probe, commands, setup, attachment, state, race, dispatched, errors = live_coordinator(tmp_path)
    if admission == "prepare_motion":
        state = probe.begin_refinement(state.context, fully_custom=True)
    external = CommandRequest.create(
        SemanticCommand(command_id=CommandID.MOVE_BASE_RELATIVE),
        client_id="operator-ui", origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.EXCLUDE,
    )

    def attempt():
        with pytest.raises(RuntimeError, match="lane.*idle|lane.*busy"):
            if admission == "begin":
                probe.begin_refinement(state.context, fully_custom=True)
            else:
                probe.prepare_motion(
                    state.context, ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
                )

    _, submitted = race.run(lambda: commands.submit(external), attempt)

    assert submitted == external.request_id
    assert errors == []
    assert dispatched == [external]
    assert not probe.refinement_controller.has_context(state.context)
    assert attachment.active_reservations == (admission == "prepare_motion")
    assert len(setup.contexts) == 1


@pytest.mark.parametrize("close_kind", ["abort", "context", "coordinator"])
def test_feedback_racing_close_retains_owned_motion_until_confirmed_stop(
    tmp_path, monkeypatch, close_kind,
):
    probe, commands, setup, attachment, state, race, dispatched, errors = live_coordinator(tmp_path)
    state = probe.begin_refinement(state.context, fully_custom=True)
    operation = probe.prepare_motion(
        state.context, ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
    )
    probe.submit_motion(operation)
    workflow_ready = None
    if close_kind == "coordinator":
        workflow_ready = Event()
        remove_listener = commands.remove_status_listener

        def remove_then_deliver_feedback(listener):
            remove_listener(listener)
            # Race cancellation with the owned operation callback that remains
            # attached after shutdown stops observing external commands.
            race.contending.clear()
            workflow_ready.set()
            assert race.entered.wait(4), "Owned feedback did not start"
            # Passive shutdown no longer enters command cancellation. Release
            # feedback once listener removal has completed its command access.
            race.contending.set()

        monkeypatch.setattr(commands, "remove_status_listener", remove_then_deliver_feedback)

    def close():
        if close_kind == "abort":
            return probe.end_refinement(state.context)
        if close_kind == "context":
            return probe.close_context(state.context)
        return probe.close()

    _, received = race.run(
        lambda: commands.handle_execution_status(execution_status(
            operation.request_id, CommandControllerState.RUNNING,
        )), close, workflow_ready,
    )

    assert received is True
    assert errors == []
    expected = [CommandID.MOVE_SAFE_APPROACH]
    if close_kind != "coordinator":
        expected.append(CommandID.EMERGENCY_CANCEL)
        assert dispatched[-1].stop_request_ids == (operation.request_id,)
    assert [request.command.command_id for request in dispatched] == expected
    assert probe.refinement_controller.has_context(state.context)
    assert attachment.active_reservations == 1
    assert setup.contexts == (state.context,)

    assert commands.handle_execution_status(execution_status(
        operation.request_id, CommandControllerState.STOP_UNCONFIRMED, 2,
    ))
    assert probe.refinement_controller.has_context(state.context)
    assert attachment.active_reservations == 1
    with pytest.raises(RuntimeError, match="closing|cancellation|closed|lane.*idle"):
        probe.prepare_motion(
            state.context, ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
        )

    assert commands.handle_execution_status(execution_status(
        operation.request_id, CommandControllerState.STOP_CONFIRMED, 3,
    ))
    assert errors == []
    assert not probe.refinement_controller.has_context(state.context)
    assert attachment.active_reservations == 0
    if close_kind == "abort":
        context = probe.context(state.context.context_id, "probe-ui")
        assert probe.snapshot(context).refinement is None
    else:
        assert setup.contexts == ()
        with pytest.raises(LookupError):
            probe.context(state.context.context_id, "probe-ui")


def test_completed_abort_cannot_cancel_a_newer_refinement(tmp_path, monkeypatch):
    probe, commands, setup, attachment, state, _, dispatched, errors = live_coordinator(tmp_path)
    state = probe.begin_refinement(state.context, fully_custom=True)
    motion = ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH)
    original = probe.prepare_motion(state.context, motion)
    probe.submit_motion(original)
    cancel_started, resume_cancel = Event(), Event()
    cancel_operation = setup.cancel_operation

    def delayed_cancel(context, request_id):
        cancel_started.set()
        assert resume_cancel.wait(4), "Terminal feedback could not finish the abort"
        return cancel_operation(context, request_id)

    monkeypatch.setattr(setup, "cancel_operation", delayed_cancel)
    with ThreadPoolExecutor(max_workers=2) as workers:
        abort = workers.submit(probe.end_refinement, state.context)
        try:
            assert cancel_started.wait(4)
            assert workers.submit(
                commands.handle_execution_status,
                execution_status(original.request_id, CommandControllerState.SUCCEEDED),
            ).result(timeout=2)
            context = probe.context(state.context.context_id, "probe-ui")
            assert probe.snapshot(context).refinement is None
            assert attachment.active_reservations == 0

            newer = probe.begin_refinement(context, fully_custom=True)
            replacement = probe.prepare_motion(newer.context, motion)
            probe.submit_motion(replacement)
        finally:
            resume_cancel.set()
        returned = abort.result(timeout=5)

    assert errors == []
    assert returned.context == newer.context
    assert returned.refinement is not None
    assert commands.active_request_id == replacement.request_id
    assert probe.refinement_controller.request_ids_for(newer.context) == (replacement.request_id,)
    assert attachment.active_reservations == 1
    assert dispatched == [original.request, replacement.request]
    assert not commands.software_stop_status.latched
