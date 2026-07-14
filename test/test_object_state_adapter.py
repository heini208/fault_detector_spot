"""Tests for inspection object ROS message conversion."""

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from fault_detector_msgs.msg import (
    InspectionObjectState,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
    ResolvedObjectPose,
)
from fault_detector_spot.inspection.object_state_adapter import (
    resolved_object_pose_to_msg,
    resolved_object_poses_to_array_msg,
)


def create_pose(
    x: float,
    frame_id: str = "map",
) -> PoseStamped:
    """Create a valid stamped pose."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def test_live_object_conversion():
    """Live object state includes all available poses."""
    remembered = create_pose(1.0)
    live = create_pose(2.0)

    resolved = ResolvedObjectPose(
        object_id="motor_a",
        tag_id=23,
        state=ObjectPoseState.LIVE,
        selected_pose=live,
        remembered_pose=remembered,
        live_pose=live,
        message="Using live marker observation",
    )

    message = resolved_object_pose_to_msg(
        resolved
    )

    assert message.object_id == "motor_a"
    assert message.tag_id == 23
    assert message.state == InspectionObjectState.LIVE

    assert message.has_selected_pose
    assert message.has_remembered_pose
    assert message.has_live_pose

    assert message.selected_pose.pose.position.x == 2.0
    assert message.remembered_pose.pose.position.x == 1.0
    assert message.live_pose.pose.position.x == 2.0


def test_unavailable_object_conversion():
    """Unavailable objects contain no valid poses."""
    resolved = ResolvedObjectPose(
        object_id="motor_b",
        tag_id=None,
        state=ObjectPoseState.UNAVAILABLE,
        message="No pose available",
    )

    message = resolved_object_pose_to_msg(
        resolved
    )

    assert message.tag_id == -1
    assert (
        message.state
        == InspectionObjectState.UNAVAILABLE
    )
    assert not message.has_selected_pose
    assert not message.has_remembered_pose
    assert not message.has_live_pose


def test_array_conversion_is_sorted():
    """Array messages use deterministic object ordering."""
    first = ResolvedObjectPose(
        object_id="motor_b",
        tag_id=2,
        state=ObjectPoseState.REMEMBERED,
        selected_pose=create_pose(2.0),
        remembered_pose=create_pose(2.0),
    )

    second = ResolvedObjectPose(
        object_id="motor_a",
        tag_id=1,
        state=ObjectPoseState.REMEMBERED,
        selected_pose=create_pose(1.0),
        remembered_pose=create_pose(1.0),
    )

    header = Header()
    header.frame_id = "map"

    message = resolved_object_poses_to_array_msg(
        [first, second],
        error_message="test error",
        header=header,
    )

    assert [
        item.object_id
        for item in message.objects
    ] == [
        "motor_a",
        "motor_b",
    ]

    assert message.header.frame_id == "map"
    assert message.error_message == "test error"


def test_conversion_copies_pose_messages():
    """Converted messages do not share pose instances."""
    live = create_pose(3.0)

    resolved = ResolvedObjectPose(
        object_id="motor_a",
        tag_id=23,
        state=ObjectPoseState.LIVE,
        selected_pose=live,
        live_pose=live,
    )

    message = resolved_object_pose_to_msg(
        resolved
    )

    live.pose.position.x = 9.0

    assert message.selected_pose.pose.position.x == 3.0
    assert message.live_pose.pose.position.x == 3.0