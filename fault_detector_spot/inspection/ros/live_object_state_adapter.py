"""Convert a live local object result into a ROS message."""

from copy import deepcopy

from fault_detector_msgs.msg import LiveInspectionObjectState
from std_msgs.msg import Header

from fault_detector_spot.inspection.model.resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)


def live_object_pose_to_msg(
    resolved: ResolvedObjectPose,
    header: Header,
) -> LiveInspectionObjectState:
    """Convert one mapless object result into its state message."""
    message = LiveInspectionObjectState()
    message.header = deepcopy(header)
    message.object_id = resolved.object_id
    message.tag_id = (
        resolved.tag_id
        if resolved.tag_id is not None
        else -1
    )
    message.execution_frame = resolved.frame_id
    message.observation_source = resolved.observation_source
    message.observation_age_sec = (
        resolved.observation_age_sec
        if resolved.observation_age_sec is not None
        else -1.0
    )

    if resolved.observation_timestamp is not None:
        message.observation_timestamp = deepcopy(
            resolved.observation_timestamp
        )

    if resolved.state == ObjectPoseState.LIVE:
        message.state = LiveInspectionObjectState.LIVE
    elif resolved.state == ObjectPoseState.UNAVAILABLE:
        message.state = LiveInspectionObjectState.UNAVAILABLE
    else:
        message.state = LiveInspectionObjectState.INVALID

    if resolved.selected_pose is not None:
        message.has_object_pose = True
        message.object_pose = deepcopy(resolved.selected_pose)

    message.error_message = (
        ""
        if resolved.state == ObjectPoseState.LIVE
        else resolved.message
    )
    return message
