"""Validate tag tracking outside the behavior-tree process."""

import tf2_ros
from apriltag_msgs.msg import (
    AprilTagDetection,
    AprilTagDetectionArray,
)
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.sensing.observations.tag_observation_tracker import (
    BaseTagObservationTracker,
    HandTagObservationTracker,
    merge_tag_observations,
)


class FakeTfBuffer:
    def __init__(self, transforms=None):
        self.transforms = transforms or {}

    def lookup_transform(self, target_frame, source_frame, lookup_time):
        value = self.transforms.get(source_frame)
        if value is None:
            raise tf2_ros.LookupException(source_frame)
        return value


def transform(stamp_sec, x, frame_id="body"):
    value = TransformStamped()
    value.header.frame_id = frame_id
    value.header.stamp.sec = stamp_sec
    value.transform.translation.x = x
    value.transform.rotation.w = 1.0
    return value


def tf_message(*frame_ids):
    message = TFMessage()
    for frame_id in frame_ids:
        value = TransformStamped()
        value.child_frame_id = frame_id
        message.transforms.append(value)
    return message


def detection_message(tag_id=7):
    message = AprilTagDetectionArray()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 100_000_000
    detection = AprilTagDetection()
    detection.id = tag_id
    detection.hamming = 0
    detection.decision_margin = 100.0
    message.detections.append(detection)
    return message


def tag(tag_id, x):
    value = TagElement()
    value.id = tag_id
    value.pose.pose.position.x = x
    value.pose.pose.orientation.w = 1.0
    return value


def test_base_tracker_uses_raw_stamp_and_filtered_geometry():
    tracker = BaseTagObservationTracker(
        FakeTfBuffer({
            "fiducial_7": transform(20, 1.2),
            "filtered_fiducial_7": transform(10, 1.0),
        }),
        max_age_sec=1.5,
    )
    tracker.receive_tf_frames(tf_message("fiducial_7"))

    observations = tracker.snapshot(Time(seconds=20.1))

    assert set(observations) == {7}
    assert observations[7].pose.header.stamp.sec == 20
    assert observations[7].pose.pose.position.x == 1.0


def test_hand_tracker_retries_detection_until_tf_arrives():
    buffer = FakeTfBuffer()
    tracker = HandTagObservationTracker(
        buffer,
        max_age_sec=1.0,
        tf_pending_sec=0.5,
    )
    tracker.receive_detections(detection_message())

    assert tracker.snapshot(Time(seconds=10.2)) == {}
    assert tracker.pending_tag_ids() == (7,)

    buffer.transforms["tag36h11:7"] = transform(10, 2.0)
    observations = tracker.snapshot(Time(seconds=10.3))

    assert set(observations) == {7}
    assert tracker.pending_tag_ids() == ()


def test_base_observation_has_priority_over_hand_fallback():
    observations = merge_tag_observations(
        {7: tag(7, 1.0)},
        {7: tag(7, 2.0), 8: tag(8, 3.0)},
    )

    assert set(observations) == {7, 8}
    assert observations[7].pose.pose.position.x == 1.0
    assert observations[8].pose.pose.position.x == 3.0
