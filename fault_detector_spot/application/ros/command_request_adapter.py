"""Convert application command requests to and from ROS messages."""

from fault_detector_msgs.msg import CommandRequest as CommandRequestMessage

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


def command_request_from_message(
    message: CommandRequestMessage,
) -> CommandRequest[SemanticCommand]:
    """Convert one ROS request into the application command model."""
    if not isinstance(message, CommandRequestMessage):
        raise TypeError("Expected a CommandRequest message")
    request = CommandRequest(
        request_id=message.request_id,
        client_id=message.client_id,
        context_id=message.context_id,
        origin=message.origin,
        recording_policy=message.recording_policy,
        command=semantic_command_from_message(message.payload),
        stop_request_ids=tuple(message.stop_request_ids),
        execution_instance_id=message.execution_instance_id,
        stop_execution_instance_ids=tuple(message.stop_execution_instance_ids),
    )
    return request


def command_request_to_message(
    request: CommandRequest[SemanticCommand],
) -> CommandRequestMessage:
    """Convert one semantic application request to a ROS message."""
    if not isinstance(request, CommandRequest):
        raise TypeError("Expected an application CommandRequest")
    if not isinstance(request.command, SemanticCommand):
        raise TypeError(
            "Command request must contain SemanticCommand"
        )
    message = CommandRequestMessage()
    message.request_id = request.request_id
    message.client_id = request.client_id
    message.context_id = request.context_id
    message.origin = int(request.origin)
    message.recording_policy = int(request.recording_policy)
    message.payload = semantic_command_to_message(request.command)
    message.stop_request_ids = list(request.stop_request_ids)
    message.execution_instance_id = request.execution_instance_id
    message.stop_execution_instance_ids = list(request.stop_execution_instance_ids)
    return message


__all__ = [
    "command_request_from_message",
    "command_request_to_message",
]
