"""Tests for the ROS command request adapter."""

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
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
    command_request_to_message,
)


def make_request():
    """Build one valid semantic setup request."""
    return CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.MOVE_ARM_RELATIVE),
        client_id="probe_ui",
        context_id="probe_setup_4",
        origin=CommandOrigin.PROBE_SETUP,
        recording_policy=RecordingPolicy.EXCLUDE,
    )


def test_adapter_round_trip_preserves_request_metadata():
    request = make_request()

    restored = command_request_from_message(
        command_request_to_message(request)
    )

    assert restored.request_id == request.request_id
    assert restored.client_id == "probe_ui"
    assert restored.context_id == "probe_setup_4"
    assert restored.origin is CommandOrigin.PROBE_SETUP
    assert restored.recording_policy is RecordingPolicy.EXCLUDE
    assert isinstance(restored.command, SemanticCommand)
    assert restored.command.command_id is CommandID.MOVE_ARM_RELATIVE


def test_adapter_rejects_conflicting_nested_request_identity():
    message = command_request_to_message(make_request())
    message.command.command.request_id = (
        "00000000-0000-4000-8000-000000000001"
    )

    with pytest.raises(ValueError, match="request IDs must match"):
        command_request_from_message(message)


def test_adapter_rejects_empty_command_id():
    message = command_request_to_message(make_request())
    message.command.command.command_id = ""

    with pytest.raises(ValueError, match="Command ID"):
        command_request_from_message(message)


def test_adapter_rejects_non_semantic_outbound_payload():
    request = CommandRequest.create(
        command=object(),
        client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=(
            RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
        ),
    )

    with pytest.raises(TypeError, match="SemanticCommand"):
        command_request_to_message(request)
