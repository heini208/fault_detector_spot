from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped

from fault_detector_spot.behaviour_tree.nodes.sensing.hand_camera_tag_detection import (
    HandCameraTagDetection,
)


def test_tag_element_preserves_orientation_and_stamp():
    transform = TransformStamped()
    transform.header.frame_id = "body"
    transform.transform.rotation.x = 0.1
    transform.transform.rotation.y = 0.2
    transform.transform.rotation.z = 0.3
    transform.transform.rotation.w = 0.9

    stamp = Time(sec=12, nanosec=34)

    tag = HandCameraTagDetection._create_tag_element(
        7,
        transform,
        stamp,
    )

    assert tag.pose.header.frame_id == "body"
    assert tag.pose.header.stamp == stamp
    assert tag.pose.pose.orientation.x == 0.1
    assert tag.pose.pose.orientation.y == 0.2
    assert tag.pose.pose.orientation.z == 0.3
    assert tag.pose.pose.orientation.w == 0.9