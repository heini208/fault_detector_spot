"""Tests for the ROS command request adapter."""

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
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


def test_wire_payload_does_not_duplicate_request_identity():
    message = command_request_to_message(make_request())

    assert message.request_id
    assert not hasattr(message.payload, "request_id")


def test_wire_payload_flattens_inspection_selection():
    request = CommandRequest.create(
        command=SemanticCommand(
            command_id=CommandID.EXECUTE_PROBE_POINT,
            inspection=InspectionSelection(
                object_id="pump",
                routine_id="bearing",
                probe_point_id="point_1",
            ),
        ),
        client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )

    message = command_request_to_message(request)
    restored = command_request_from_message(message)

    assert not hasattr(message.payload, "inspection")
    assert message.payload.object_id == "pump"
    assert message.payload.routine_id == "bearing"
    assert message.payload.probe_point_id == "point_1"
    assert restored.command.inspection == request.command.inspection


def test_adapter_rejects_empty_command_id():
    message = command_request_to_message(make_request())
    message.payload.command_id = ""

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
