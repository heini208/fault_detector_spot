"""Tests for application-level operational request ownership."""

import pytest

from fault_detector_msgs.msg import OperationalIntent
from fault_detector_msgs.msg import ComplexCommand

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
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
    """Expose the command-controller surface used by the application."""

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
    """Return one valid public intent."""
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
    assert operation.request.command.command.command_id == (
        CommandID.STAND_UP.value
    )


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
    command = ComplexCommand()
    command.command.command_id = CommandID.STOW_ARM.value
    request = CommandRequest.create(
        command=command,
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
