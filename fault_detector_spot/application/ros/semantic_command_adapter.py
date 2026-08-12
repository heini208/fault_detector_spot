"""Convert semantic commands to and from the legacy ROS wire DTO."""

from fault_detector_msgs.msg import ComplexCommand
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    InspectionSelection,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)


def stamped_pose_from_message(message: PoseStamped) -> StampedPose:
    """Convert one ROS pose message into application data."""
    if not isinstance(message, PoseStamped):
        raise TypeError("Expected a PoseStamped message")
    return StampedPose(
        frame_id=message.header.frame_id,
        stamp_sec=int(message.header.stamp.sec),
        stamp_nanosec=int(message.header.stamp.nanosec),
        position=CommandVector3(
            x=message.pose.position.x,
            y=message.pose.position.y,
            z=message.pose.position.z,
        ),
        orientation=CommandQuaternion(
            x=message.pose.orientation.x,
            y=message.pose.orientation.y,
            z=message.pose.orientation.z,
            w=message.pose.orientation.w,
        ),
    )


def stamped_pose_to_message(value: StampedPose) -> PoseStamped:
    """Convert application pose data to a ROS pose message."""
    if not isinstance(value, StampedPose):
        raise TypeError("Expected a StampedPose")
    message = PoseStamped()
    message.header.frame_id = value.frame_id
    message.header.stamp.sec = value.stamp_sec
    message.header.stamp.nanosec = value.stamp_nanosec
    message.pose.position.x = value.position.x
    message.pose.position.y = value.position.y
    message.pose.position.z = value.position.z
    message.pose.orientation.x = value.orientation.x
    message.pose.orientation.y = value.orientation.y
    message.pose.orientation.z = value.orientation.z
    message.pose.orientation.w = value.orientation.w
    return message


def semantic_command_from_message(
    message: ComplexCommand,
) -> SemanticCommand:
    """Convert the internal ROS wire DTO into a semantic command."""
    if not isinstance(message, ComplexCommand):
        raise TypeError("Expected a ComplexCommand message")
    command_id = message.command.command_id.strip()
    if not command_id:
        raise ValueError("Command ID must not be empty")

    tag = None
    tag_pose = message.tag.pose
    if (
        int(message.tag.id) != 0
        or bool(tag_pose.header.frame_id.strip())
        or int(tag_pose.header.stamp.sec) != 0
        or int(tag_pose.header.stamp.nanosec) != 0
        or any((
            tag_pose.pose.position.x,
            tag_pose.pose.position.y,
            tag_pose.pose.position.z,
            tag_pose.pose.orientation.x,
            tag_pose.pose.orientation.y,
            tag_pose.pose.orientation.z,
            tag_pose.pose.orientation.w,
        ))
    ):
        tag = SemanticTag(
            id=int(message.tag.id),
            pose=stamped_pose_from_message(tag_pose),
        )

    return SemanticCommand(
        command_id=command_id,
        tag=tag,
        offset=stamped_pose_from_message(message.offset),
        orientation_mode=message.orientation_mode,
        wait_time=message.wait_time,
        map_name=message.map_name,
        waypoint_name=message.waypoint_name,
        inspection=InspectionSelection(
            object_id=message.inspection.object_id,
            routine_id=message.inspection.routine_id,
            probe_point_id=message.inspection.probe_point_id,
        ),
    )


def semantic_command_to_message(
    command: SemanticCommand,
    request_id: str = "",
) -> ComplexCommand:
    """Convert one semantic command into the ROS wire DTO."""
    if not isinstance(command, SemanticCommand):
        raise TypeError("Expected a SemanticCommand")
    message = ComplexCommand()
    message.command.command_id = command.command_id.value
    message.command.request_id = request_id.strip()

    if command.tag is not None:
        message.tag.id = command.tag.id
        message.tag.pose = stamped_pose_to_message(command.tag.pose)

    message.offset = stamped_pose_to_message(command.offset)
    message.orientation_mode = command.orientation_mode
    message.wait_time = float(command.wait_time)
    message.map_name = command.map_name
    message.waypoint_name = command.waypoint_name
    message.inspection.object_id = command.inspection.object_id
    message.inspection.routine_id = command.inspection.routine_id
    message.inspection.probe_point_id = (
        command.inspection.probe_point_id
    )
    return message


__all__ = [
    "semantic_command_from_message",
    "semantic_command_to_message",
    "stamped_pose_from_message",
    "stamped_pose_to_message",
]
