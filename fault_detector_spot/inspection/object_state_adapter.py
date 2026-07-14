"""Convert resolved inspection objects into ROS messages."""

from copy import deepcopy
from typing import Iterable, Optional

from fault_detector_msgs.msg import (
    InspectionObjectState,
    InspectionObjectStateArray,
)
from std_msgs.msg import Header

from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
    ResolvedObjectPose,
)


STATE_TO_MESSAGE = {
    ObjectPoseState.LIVE:
        InspectionObjectState.LIVE,
    ObjectPoseState.REMEMBERED:
        InspectionObjectState.REMEMBERED,
    ObjectPoseState.UNAVAILABLE:
        InspectionObjectState.UNAVAILABLE,
    ObjectPoseState.INVALID:
        InspectionObjectState.INVALID,
}


def resolved_object_pose_to_msg(
    resolved: ResolvedObjectPose,
) -> InspectionObjectState:
    """Convert one resolved object into a ROS message."""
    message = InspectionObjectState()

    message.object_id = resolved.object_id
    message.tag_id = (
        resolved.tag_id
        if resolved.tag_id is not None
        else -1
    )
    message.state = STATE_TO_MESSAGE[
        resolved.state
    ]
    message.message = resolved.message

    if resolved.selected_pose is not None:
        message.has_selected_pose = True
        message.selected_pose = deepcopy(
            resolved.selected_pose
        )

    if resolved.remembered_pose is not None:
        message.has_remembered_pose = True
        message.remembered_pose = deepcopy(
            resolved.remembered_pose
        )

    if resolved.live_pose is not None:
        message.has_live_pose = True
        message.live_pose = deepcopy(
            resolved.live_pose
        )

    return message


def resolved_object_poses_to_array_msg(
    resolved_objects: Iterable[
        ResolvedObjectPose
    ],
    error_message: str = "",
    header: Optional[Header] = None,
) -> InspectionObjectStateArray:
    """Convert multiple resolved objects into an array message."""
    message = InspectionObjectStateArray()

    if header is not None:
        message.header = deepcopy(header)

    ordered_objects = sorted(
        resolved_objects,
        key=lambda resolved: resolved.object_id,
    )

    message.objects = [
        resolved_object_pose_to_msg(resolved)
        for resolved in ordered_objects
    ]
    message.error_message = error_message

    return message