"""Tests for application-level operational request ownership."""

from types import SimpleNamespace

import pytest

from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
    CommandControllerStatus,
    UnknownCommandRequest,
)


class FakeCommandController:
    def __init__(self):
        self.listeners = []
        self.submitted = []
        self.cancelled = []
        self.emergency_clients = []

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def cancel(self, request_id):
        self.cancelled.append(request_id)
        return request_id

    def cancel_all(self, client_id):
        self.emergency_clients.append(client_id)
        return "emergency-request"

    def emit(self, request, state):
        status = CommandControllerStatus(request=request, state=state)
        for listener in tuple(self.listeners):
            listener(status)


def stand_intent():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_STAND_UP
    return intent


def test_prepares_operational_request_with_server_owned_policy():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)

    operation = controller.prepare_operation(
        stand_intent(),
        "operator-ui",
        "manual-control",
    )

    assert operation.intent == OperationalIntent.INTENT_STAND_UP
    assert operation.request.client_id == "operator-ui"
    assert operation.request.context_id == "manual-control"
    assert operation.request.origin is CommandOrigin.OPERATIONAL
    assert operation.request.recording_policy is (
        RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
    )
    assert isinstance(operation.request.command, SemanticCommand)
    assert operation.request.command.command_id is CommandID.STAND_UP


def test_submit_routes_only_prepared_operation():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    operation = controller.prepare_operation(stand_intent(), "operator-ui")

    request_id = controller.submit(operation)

    assert request_id == operation.request_id
    assert command_controller.submitted == [operation.request]


def test_status_is_exposed_with_application_operation_identity():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    statuses = []
    controller.add_status_listener(statuses.append)
    operation = controller.prepare_operation(stand_intent(), "operator-ui")
    controller.submit(operation)

    command_controller.emit(
        operation.request,
        CommandControllerState.RUNNING,
    )

    assert statuses[-1].operation == operation
    assert statuses[-1].state is CommandControllerState.RUNNING


def test_client_cannot_cancel_another_clients_operation():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    operation = controller.prepare_operation(stand_intent(), "operator-ui")
    controller.submit(operation)

    with pytest.raises(ValueError, match="does not own"):
        controller.cancel("different-ui", operation.request_id)

    assert command_controller.cancelled == []


def test_untracked_semantic_request_still_reaches_state_listeners():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    statuses = []
    controller.add_status_listener(statuses.append)
    request = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.STOW_ARM),
        client_id="recording-playback",
        origin=CommandOrigin.PLAYBACK,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )

    command_controller.emit(request, CommandControllerState.RUNNING)

    assert statuses[-1].operation.intent == (
        OperationalIntent.INTENT_UNSPECIFIED
    )
    assert statuses[-1].operation.request == request

    with pytest.raises(UnknownCommandRequest):
        controller.cancel("operator-ui", request.request_id)


def test_emergency_stop_uses_validated_remote_client_identity():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)

    request_id = controller.emergency_stop(" operator-ui ")

    assert request_id == "emergency-request"
    assert command_controller.emergency_clients == ["operator-ui"]


def test_close_detaches_from_command_controller():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)

    controller.close()

    assert command_controller.listeners == []


def test_application_controller_owns_shared_setup_coordinator():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)

    context = controller.setup_coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )

    assert controller.setup_coordinator.is_current(context)


def test_probe_setup_attachment_uses_shared_coordinator():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    probe = SimpleNamespace(
        uses_setup_coordinator=lambda coordinator: (
            coordinator is controller.setup_coordinator
        ),
        close=lambda: None,
    )

    controller.attach_probe_setup(probe)

    assert controller.probe_setup_coordinator is probe

    with pytest.raises(RuntimeError, match="already attached"):
        controller.attach_probe_setup(probe)


def test_setup_status_is_owned_only_by_setup_coordinator():
    command_controller = FakeCommandController()
    controller = ApplicationController(command_controller)
    application_statuses = []
    setup_statuses = []
    controller.add_status_listener(application_statuses.append)
    context = controller.setup_coordinator.open_context(
        CommandOrigin.NAVIGATION_SETUP,
        "navigation-ui",
    )
    controller.setup_coordinator.add_operation_listener(
        context,
        setup_statuses.append,
    )
    operation = controller.setup_coordinator.prepare_command(
        context,
        SemanticCommand(command_id=CommandID.STOW_ARM),
    )
    controller.setup_coordinator.submit(operation)

    command_controller.emit(
        operation.request,
        CommandControllerState.RUNNING,
    )

    assert application_statuses == []
    assert setup_statuses[-1].operation == operation

