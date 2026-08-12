"""Tests for shared setup context and command ownership."""

from dataclasses import FrozenInstanceError

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
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

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

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
    probe_operation = coordinator.prepare_command(
        probe,
        command(CommandID.READY_ARM),
    )
    coordinator.submit(navigation_operation)
    coordinator.submit(probe_operation)

    controller.emit(
        navigation_operation.request,
        CommandControllerState.RUNNING,
    )

    assert [status.operation for status in navigation_statuses] == [
        navigation_operation
    ]
    assert probe_statuses == []

    controller.emit(
        probe_operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert [status.operation for status in probe_statuses] == [
        probe_operation
    ]


def test_advancing_context_rejects_delayed_results():
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

    coordinator.advance_context(context)
    controller.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert statuses == []


def test_closing_context_invalidates_pending_callbacks():
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
    controller.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
    )

    assert statuses == []
    assert not coordinator.is_current(context)
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
