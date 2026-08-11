"""Tests for the local object state message adapter."""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from fault_detector_msgs.msg import LiveInspectionObjectState
from fault_detector_spot.inspection.ros.live_object_state_adapter import (
    live_object_pose_to_msg,
)
from fault_detector_spot.inspection.data.resolved_object_pose import (
    ObjectPoseState,
    ResolvedObjectPose,
)


def test_live_result_message_contains_local_metadata():
    """The local topic exposes frame, age, source, and pose."""
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.header.stamp = Time(sec=10, nanosec=1)
    pose.pose.orientation.w = 1.0
    result = ResolvedObjectPose(
        object_id="panel",
        tag_id=7,
        state=ObjectPoseState.LIVE,
        selected_pose=pose,
        frame_id="odom",
        observation_timestamp=pose.header.stamp,
        observation_age_sec=0.05,
        observation_source="base",
    )
    header = Header()
    header.frame_id = "odom"

    message = live_object_pose_to_msg(result, header)

    assert message.state == LiveInspectionObjectState.LIVE
    assert message.execution_frame == "odom"
    assert message.observation_source == "base"
    assert message.observation_age_sec == 0.05
    assert message.has_object_pose
    assert message.object_pose.header.frame_id == "odom"
