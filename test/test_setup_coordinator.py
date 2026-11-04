"""Tests for shared setup context and command ownership."""

from dataclasses import FrozenInstanceError

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
    CommandControllerStatus,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextLifecycle,
    StaleSetupContext,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)


class FakeCommandController:
    """Expose command submission and status listeners to the coordinator."""

    def __init__(self):
        self.listeners = []
        self.submitted = []
        self.cancelled = []
        self.active_request_id = ""
        self.queued_request_ids = ()

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def run_if_idle(self, action, busy_message):
        if self.active_request_id or self.queued_request_ids:
            raise RuntimeError(busy_message)
        return action()

    def run_configuration_change(self, action, busy_message):
        return self.run_if_idle(action, busy_message)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def cancel(self, request_id):
        self.cancelled.append(request_id)
        return request_id

    def emit(self, request, state, detail=""):
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
        )
        for listener in tuple(self.listeners):
            listener(status)


def command(command_id=CommandID.READY_ARM):
    """Return one immutable semantic command."""
    return SemanticCommand(command_id=command_id)


def test_navigation_and_probe_contexts_coexist_without_global_lock():
    coordinator = SetupCoordinator(FakeCommandController())

    navigation = coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )
    probe = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )

    assert coordinator.contexts == (navigation, probe)
    assert navigation.context_id != probe.context_id
    assert coordinator.is_current(navigation)
    assert coordinator.is_current(probe)


def test_context_snapshots_are_immutable_and_revisioned():
    coordinator = SetupCoordinator(FakeCommandController())
    events = []
    coordinator.add_context_listener(events.append)
    initial = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )

    updated = coordinator.advance_context(initial)

    assert updated.context_id == initial.context_id
    assert updated.revision == 1
    assert not coordinator.is_current(initial)
    assert coordinator.is_current(updated)
    assert [event.lifecycle for event in events] == [
        SetupContextLifecycle.OPENED,
        SetupContextLifecycle.UPDATED,
    ]
    with pytest.raises(FrozenInstanceError):
        updated.revision = 2


def test_only_setup_origins_can_open_contexts():
    coordinator = SetupCoordinator(FakeCommandController())

    with pytest.raises(ValueError, match="Unsupported setup origin"):
        coordinator.open_context(
            CommandOrigin.OPERATIONAL,
            "operator-ui",
        )


def test_setup_request_metadata_is_owned_by_coordinator():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        " probe-ui ",
    )
    source = command()

    operation = coordinator.prepare_command(context, source)

    with pytest.raises(FrozenInstanceError):
        source.command_id = CommandID.STOW_ARM

    request_id = coordinator.submit(operation)

    assert request_id == operation.request_id
    assert controller.submitted == [operation.request]
    assert operation.request.client_id == "probe-ui"
    assert operation.request.context_id == context.context_id
    assert operation.request.origin is CommandOrigin.PROBE_SETUP
    assert operation.request.recording_policy is RecordingPolicy.EXCLUDE
    assert operation.request.command.command_id is CommandID.READY_ARM


def test_uncertain_setup_motion_keeps_ownership_and_blocks_idle_admission():
    controller = FakeCommandController()
    controller.cancel = lambda request_id: request_id
    controller.queued_request_ids = ()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe-ui")
    operation = coordinator.prepare_command(context, command())
    coordinator.submit(operation)
    controller.active_request_id = operation.request_id
    for state in (
        CommandControllerState.UNKNOWN,
        CommandControllerState.STOP_REQUESTED,
        CommandControllerState.STOP_UNCONFIRMED,
    ):
        controller.emit(operation.request, state)
        assert coordinator.cancel_operation(context, operation.request_id) == operation.request_id
        with pytest.raises(RuntimeError, match="Busy"):
            coordinator.require_command_lane_idle("Busy")
        with pytest.raises(RuntimeError, match="Busy"):
            coordinator.require_configuration_idle("Busy")

    controller.active_request_id = ""
    controller.emit(operation.request, CommandControllerState.STOP_CONFIRMED)

    coordinator.require_command_lane_idle("Busy")
    coordinator.require_configuration_idle("Busy")
    with pytest.raises(LookupError, match="Unknown setup operation"):
        coordinator.cancel_operation(context, operation.request_id)


@pytest.mark.parametrize("reconciled", [False, True])
def test_configuration_ignores_idle_stop_but_workflow_admission_remains_blocked(reconciled):
    sent = []
    controller = CommandController(dispatch_request=sent.append)
    coordinator = SetupCoordinator(controller)
    stop_id = controller.cancel_all("startup_reconciliation")
    if reconciled:
        controller.poll()
    assert controller.queued_request_ids == ()
    assert controller.active_request_id == stop_id
    stop_before = controller.software_stop_status

    assert coordinator.run_configuration_change(lambda: "saved", "Busy") == "saved"
    coordinator.require_configuration_idle("Busy", "Queued work")
    with pytest.raises(RuntimeError, match="Software stop is pending or unconfirmed"):
        coordinator.run_if_command_lane_idle(lambda: pytest.fail("Started a workflow"), "Busy")
    with pytest.raises(RuntimeError, match="Software stop is pending or unconfirmed"):
        coordinator.require_command_lane_idle("Busy")

    assert controller.software_stop_status == stop_before
    assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL]


@pytest.mark.parametrize("submitted", [False, True])
def test_configuration_keeps_prepared_and_submitted_setup_ownership(submitted):
    controller = CommandController(dispatch_request=lambda request: None)
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe-ui")
    operation = coordinator.prepare_command(context, command())
    if submitted:
        coordinator.submit(operation)
    controller.cancel_all()
    controller.poll()
    assert controller.queued_request_ids == ()
    with pytest.raises(RuntimeError, match="Busy"):
        coordinator.run_configuration_change(lambda: pytest.fail("Changed owned data"), "Busy")
    with pytest.raises(RuntimeError, match="Busy"):
        coordinator.require_configuration_idle("Busy")


def test_setup_coordinator_rejects_ros_or_untyped_command_payloads():
    coordinator = SetupCoordinator(FakeCommandController())
    context = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )

    with pytest.raises(TypeError, match="SemanticCommand"):
        coordinator.prepare_command(context, object())


def test_context_listeners_receive_only_their_own_results():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    navigation = coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )
    probe = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )
    navigation_statuses = []
    probe_statuses = []
    coordinator.add_operation_listener(
        navigation,
        navigation_statuses.append,
    )
    coordinator.add_operation_listener(probe, probe_statuses.append)
    navigation_operation = coordinator.prepare_command(
        navigation,
        command(CommandID.START_SLAM),
    )
    coordinator.submit(navigation_operation)

    controller.emit(
        navigation_operation.request,
        CommandControllerState.RUNNING,
    )

    assert [status.operation for status in navigation_statuses] == [
        navigation_operation
    ]
    assert probe_statuses == []

    controller.emit(navigation_operation.request, CommandControllerState.SUCCEEDED)
    probe_operation = coordinator.prepare_command(probe, command(CommandID.READY_ARM))
    coordinator.submit(probe_operation)

    controller.emit(
        probe_operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert [status.operation for status in probe_statuses] == [
        probe_operation
    ]


def test_advancing_context_cannot_discard_pending_ownership():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )
    statuses = []
    coordinator.add_operation_listener(context, statuses.append)
    operation = coordinator.prepare_command(context, command())
    coordinator.submit(operation)

    with pytest.raises(RuntimeError, match="active operation"):
        coordinator.advance_context(context)
    controller.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert [status.state for status in statuses] == [CommandControllerState.SUCCEEDED]
    assert coordinator.advance_context(context).revision == 1


def test_closing_context_preserves_terminal_callback_and_cancels_once():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(
        CommandOrigin.PROBE_SETUP,
        "probe-ui",
    )
    statuses = []
    coordinator.add_operation_listener(context, statuses.append)
    operation = coordinator.prepare_command(context, command())
    coordinator.submit(operation)

    coordinator.close_context(context)
    coordinator.close_context(context)
    assert controller.cancelled == [operation.request_id]
    assert coordinator.is_context_closing(context)
    assert coordinator.is_current(context)
    with pytest.raises(RuntimeError, match="admission is closed"):
        coordinator.prepare_command(context, command())
    controller.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert [status.state for status in statuses] == [CommandControllerState.SUCCEEDED]
    controller.emit(operation.request, CommandControllerState.SUCCEEDED)
    assert len(statuses) == 1
    assert not coordinator.is_current(context)
    assert coordinator.is_context_closing(context)
    with pytest.raises(StaleSetupContext):
        coordinator.require_current(context)


def test_replaced_context_rejects_operations_prepared_for_old_identity():
    coordinator = SetupCoordinator(FakeCommandController())
    original = coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )
    operation = coordinator.prepare_command(original, command())

    replacement = coordinator.replace_context(original)

    assert replacement.context_id != original.context_id
    with pytest.raises(ValueError, match="not prepared"):
        coordinator.submit(operation)


def test_close_invalidates_contexts_and_detaches_status_listener():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )

    coordinator.close()

    assert controller.listeners == []
    assert coordinator.contexts == ()
    assert not coordinator.is_current(context)


def test_close_does_not_deadlock_with_command_status_delivery(monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    status_started, removal_started = Event(), Event()
    listener_errors = []
    controller = CommandController(listener_error_handler=listener_errors.append)

    def pause_status_delivery(_status):
        status_started.set()
        assert removal_started.wait(5)

    controller.add_status_listener(pause_status_delivery)
    coordinator = SetupCoordinator(controller)
    coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe-ui")
    request = CommandRequest.create(
        command=command(), client_id="operator-ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.EXCLUDE,
    )
    removed = []
    original_remove = controller.remove_status_listener

    def remove_listener(listener):
        removal_started.set()
        # Bound the real command-lock wait so a regression cannot hang pytest.
        if not controller._lock.acquire(timeout=2):
            raise TimeoutError("Listener removal deadlocked with status delivery")
        try:
            original_remove(listener)
            removed.append(listener)
        finally:
            controller._lock.release()

    monkeypatch.setattr(controller, "remove_status_listener", remove_listener)
    with ThreadPoolExecutor(max_workers=2) as workers:
        status_delivery = workers.submit(controller.submit, request)
        try:
            assert status_started.wait(5)
            shutdown = workers.submit(coordinator.close)
            shutdown.result(timeout=5)
        finally:
            removal_started.set()
        assert status_delivery.result(timeout=5) == request.request_id

    assert listener_errors == []
    assert coordinator.contexts == ()
    assert removed == [coordinator._handle_command_status]
    assert coordinator._handle_command_status not in controller._listeners


def test_concurrent_and_repeated_close_detach_listener_once(monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    removal_started, release_removal = Event(), Event()
    removed = []
    original_remove = controller.remove_status_listener

    def delayed_remove(listener):
        removed.append(listener)
        removal_started.set()
        assert release_removal.wait(5)
        original_remove(listener)

    monkeypatch.setattr(controller, "remove_status_listener", delayed_remove)
    with ThreadPoolExecutor(max_workers=2) as workers:
        first_close = workers.submit(coordinator.close)
        try:
            assert removal_started.wait(5)
            second_close = workers.submit(coordinator.close)
            second_close.result(timeout=2)
        finally:
            release_removal.set()
        first_close.result(timeout=5)

    coordinator.close()

    assert removed == [coordinator._handle_command_status]
    assert controller.listeners == []


def test_setup_admission_bounds_prepared_and_submitted_physical_lane():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    first = coordinator.open_context(CommandOrigin.PROBE_SETUP, "first")
    second = coordinator.open_context(CommandOrigin.NAVIGATION_SETUP, "second")
    operation = coordinator.prepare_command(first, command())
    for submit in (False, True):
        if submit:
            coordinator.submit(operation)
        for context in (first, second):
            with pytest.raises(RuntimeError, match="physical lane"):
                coordinator.prepare_command(context, command())
        with pytest.raises(RuntimeError, match="Busy"):
            coordinator.require_command_lane_idle("Busy")
    controller.emit(operation.request, CommandControllerState.CANCELLED)
    assert coordinator.prepare_command(second, command()).context == second


def test_concurrent_setup_admission_has_one_physical_lane_owner():
    from concurrent.futures import ThreadPoolExecutor
    from threading import Barrier

    coordinator = SetupCoordinator(FakeCommandController())
    contexts = [coordinator.open_context(CommandOrigin.PROBE_SETUP, name)
                for name in ("first", "second")]
    barrier = Barrier(2)

    def prepare(context):
        barrier.wait(timeout=5)
        try:
            coordinator.prepare_command(context, command())
            return True
        except RuntimeError:
            return False

    with ThreadPoolExecutor(max_workers=2) as workers:
        assert sorted(workers.map(prepare, contexts)) == [False, True]


@pytest.mark.parametrize("close_context", [True, False])
def test_close_before_submit_discards_reservation_without_physical_commands(close_context):
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe")
    results = []
    coordinator.add_operation_listener(context, results.append)
    operation = coordinator.prepare_command(context, command())

    close = (lambda: coordinator.close_context(context)) if close_context else coordinator.close
    close()
    close()

    assert [status.state for status in results] == [CommandControllerState.CANCELLED]
    assert controller.submitted == controller.cancelled == []
    with pytest.raises(ValueError, match="not prepared"):
        coordinator.submit(operation)


def test_close_during_submit_retains_ownership_without_cancelling_command():
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe")
    statuses = []
    coordinator.add_operation_listener(context, statuses.append)
    operation = coordinator.prepare_command(context, command())
    entered, release = Event(), Event()
    original_submit = controller.submit

    def delayed_submit(request):
        entered.set()
        assert release.wait(5)
        return original_submit(request)

    controller.submit = delayed_submit
    with ThreadPoolExecutor(max_workers=1) as workers:
        pending = workers.submit(coordinator.submit, operation)
        assert entered.wait(5)
        try:
            coordinator.close()
            assert coordinator.is_current(context)
            assert controller.listeners
            assert controller.cancelled == []
        finally:
            release.set()
        assert pending.result(timeout=5) == operation.request_id
    assert controller.cancelled == []
    controller.emit(operation.request, CommandControllerState.STOP_UNCONFIRMED)
    assert coordinator.is_current(context)
    controller.emit(operation.request, CommandControllerState.STOP_CONFIRMED)
    assert not coordinator.is_current(context)
    assert controller.listeners == []
    assert [status.state for status in statuses] == [
        CommandControllerState.STOP_UNCONFIRMED, CommandControllerState.STOP_CONFIRMED,
    ]


def test_close_never_calls_physical_cancellation_or_releases_active_ownership():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe")
    operation = coordinator.prepare_command(context, command())
    coordinator.submit(operation)
    def fail_cancel(_request_id):
        pytest.fail("Lifecycle close attempted physical cancellation")

    controller.cancel = fail_cancel
    coordinator.close()
    assert coordinator.is_current(context)
    assert controller.listeners
    coordinator.close()
    assert controller.cancelled == []
    controller.emit(operation.request, CommandControllerState.CANCELLED)
    assert coordinator.contexts == ()


def test_deferred_cancel_failure_does_not_become_false_submission_failure():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe")
    operation = coordinator.prepare_command(context, command())
    statuses = []
    coordinator.add_operation_listener(context, statuses.append)

    def submit(request):
        controller.submitted.append(request)
        coordinator.close_context(context)

    def cancel(_request_id):
        raise RuntimeError("Stop transport unavailable")

    controller.submit, controller.cancel = submit, cancel
    assert coordinator.submit(operation) == operation.request_id
    assert coordinator.is_current(context)
    assert [status.state for status in statuses] == [CommandControllerState.STOP_UNCONFIRMED]
    with pytest.raises(RuntimeError, match="Busy"):
        coordinator.require_command_lane_idle("Busy")
    controller.emit(operation.request, CommandControllerState.STOP_CONFIRMED)
    assert statuses[-1].state is CommandControllerState.STOP_CONFIRMED
    assert not coordinator.is_current(context)


def test_terminal_notification_keeps_lane_reserved_until_context_update_finishes():
    controller = FakeCommandController()
    coordinator = SetupCoordinator(controller)
    context = coordinator.open_context(CommandOrigin.PROBE_SETUP, "probe")
    operation = coordinator.prepare_command(context, command())
    coordinator.submit(operation)
    updated = []

    def finish(_status):
        with pytest.raises(RuntimeError, match="physical lane"):
            coordinator.prepare_command(context, command())
        updated.append(coordinator.advance_context(context))

    coordinator.add_operation_listener(context, finish)
    controller.emit(operation.request, CommandControllerState.SUCCEEDED)
    assert coordinator.prepare_command(updated[0], command()).context == updated[0]
