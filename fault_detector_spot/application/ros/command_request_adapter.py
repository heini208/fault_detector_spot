"""Convert application command requests to and from ROS messages."""

from copy import deepcopy

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    ComplexCommand,
)

from fault_detector_spot.application.commanding.command_request import (
    CommandRequest,
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
        raise ValueError(
            "Envelope and command request IDs must match"
        )
    normalized = deepcopy(command)
    normalized.command.command_id = command_id
    normalized.command.request_id = request_id
    return normalized


def command_request_from_message(
    message: CommandRequestMessage,
) -> CommandRequest[ComplexCommand]:
    """Validate and convert one ROS command request."""
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


def command_request_to_message(
    request: CommandRequest[ComplexCommand],
) -> CommandRequestMessage:
    """Validate and convert one application command request."""
    if not isinstance(request, CommandRequest):
        raise TypeError("Expected an application CommandRequest")
    command = _validated_complex_command(
        request.command,
        request.request_id,
    )
    message = CommandRequestMessage()
    message.request_id = request.request_id
    message.client_id = request.client_id
    message.context_id = request.context_id
    message.origin = int(request.origin)
    message.recording_policy = int(request.recording_policy)
    message.command = command
    return message
