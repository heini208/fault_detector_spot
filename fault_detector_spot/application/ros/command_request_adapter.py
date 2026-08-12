"""Convert application command requests to and from ROS messages."""

from copy import deepcopy

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    ComplexCommand,
)

from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
    semantic_command_to_message,
)


def _validated_complex_command(
    command: ComplexCommand,
    request_id: str,
) -> ComplexCommand:
    if not isinstance(command, ComplexCommand):
        raise TypeError("Command request must contain a ComplexCommand")
    command_id = command.command.command_id.strip()
    if not command_id:
        raise ValueError("Command ID must not be empty")
    nested_request_id = command.command.request_id.strip()
    if nested_request_id and nested_request_id != request_id:
        raise ValueError("Envelope and command request IDs must match")
    normalized = deepcopy(command)
    normalized.command.command_id = command_id
    normalized.command.request_id = request_id
    return normalized


def command_request_from_message(
    message: CommandRequestMessage,
) -> CommandRequest[ComplexCommand]:
    """Validate one ROS request while retaining its wire payload."""
    if not isinstance(message, CommandRequestMessage):
        raise TypeError("Expected a CommandRequest message")
    request = CommandRequest(
        request_id=message.request_id,
        client_id=message.client_id,
        context_id=message.context_id,
        origin=message.origin,
        recording_policy=message.recording_policy,
        command=message.command,
    )
    command = _validated_complex_command(
        request.command,
        request.request_id,
    )
    return CommandRequest(
        request_id=request.request_id,
        client_id=request.client_id,
        context_id=request.context_id,
        origin=request.origin,
        recording_policy=request.recording_policy,
        command=command,
    )


def semantic_command_request_from_message(
    message: CommandRequestMessage,
) -> CommandRequest[SemanticCommand]:
    """Convert one ROS request into the application command model."""
    wire_request = command_request_from_message(message)
    return CommandRequest(
        request_id=wire_request.request_id,
        client_id=wire_request.client_id,
        context_id=wire_request.context_id,
        origin=wire_request.origin,
        recording_policy=wire_request.recording_policy,
        command=semantic_command_from_message(wire_request.command),
    )


def command_request_to_message(request: CommandRequest) -> CommandRequestMessage:
    """Convert one application or wire request to a ROS message."""
    if not isinstance(request, CommandRequest):
        raise TypeError("Expected an application CommandRequest")
    if isinstance(request.command, SemanticCommand):
        command = semantic_command_to_message(
            request.command,
            request.request_id,
        )
    elif isinstance(request.command, ComplexCommand):
        command = _validated_complex_command(
            request.command,
            request.request_id,
        )
    else:
        raise TypeError(
            "Command request must contain SemanticCommand or ComplexCommand"
        )
    message = CommandRequestMessage()
    message.request_id = request.request_id
    message.client_id = request.client_id
    message.context_id = request.context_id
    message.origin = int(request.origin)
    message.recording_policy = int(request.recording_policy)
    message.command = command
    return message


__all__ = [
    "command_request_from_message",
    "command_request_to_message",
    "semantic_command_request_from_message",
]
