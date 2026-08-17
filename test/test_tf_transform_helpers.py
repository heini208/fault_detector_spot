"""Tests for shared tf2 transform conversion helpers."""

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.shared.ros.tf_transforms import (
    lookup_pose_data,
    transform_to_pose_data,
)


def transform_message():
    message = TransformStamped()
    message.header.frame_id = "target"
    message.child_frame_id = "source"
    message.transform.translation.x = 1.0
    message.transform.translation.y = -2.0
    message.transform.translation.z = 0.5
    message.transform.rotation.x = 0.0
    message.transform.rotation.y = 0.0
    message.transform.rotation.z = 0.0
    message.transform.rotation.w = 1.0
    return message


def test_transform_stamped_converts_to_pose_data():
    pose = transform_to_pose_data(transform_message())

    assert pose.position.x == pytest.approx(1.0)
    assert pose.position.y == pytest.approx(-2.0)
    assert pose.position.z == pytest.approx(0.5)
    assert pose.orientation == QuaternionData.identity()


def test_transform_conversion_rejects_invalid_quaternion():
    message = transform_message()
    message.transform.rotation.w = 0.0

    with pytest.raises(ValueError, match="Quaternion"):
        transform_to_pose_data(message)


def test_lookup_pose_data_preserves_tf2_target_source_order():
    calls = []

    class Buffer:
        def lookup_transform(
            self,
            target_frame,
            source_frame,
            lookup_time,
            timeout,
        ):
            calls.append(
                (
                    target_frame,
                    source_frame,
                    lookup_time,
                    timeout.nanoseconds,
                )
            )
            return transform_message()

    stamp = Time(seconds=12.0)
    pose = lookup_pose_data(
        Buffer(),
        "probe",
        "camera",
        lookup_time=stamp,
        timeout_sec=0.25,
    )

    assert pose.position.x == pytest.approx(1.0)
    assert calls[0][0] == "probe"
    assert calls[0][1] == "camera"
    assert calls[0][2] is stamp
    assert calls[0][3] == 250000000


def test_lookup_pose_data_rejects_empty_frame_names():
    class Buffer:
        pass

    with pytest.raises(ValueError, match="target frame"):
        lookup_pose_data(Buffer(), "", "camera")

    with pytest.raises(ValueError, match="source frame"):
        lookup_pose_data(Buffer(), "probe", "")
