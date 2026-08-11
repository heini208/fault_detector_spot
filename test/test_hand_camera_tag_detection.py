"""Tests for stable hand-camera AprilTag handling."""

import py_trees
import tf2_ros
from apriltag_msgs.msg import (
    AprilTagDetection,
    AprilTagDetectionArray,
)
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time

from fault_detector_spot.sensing.behaviours import (
    hand_camera_tag_detection,
)

HandCameraTagDetection = (
    hand_camera_tag_detection.HandCameraTagDetection
)


class FakeClock:
    """Provide controllable ROS time."""

    def __init__(self, seconds: float):
        self.current_time = Time(seconds=seconds)
        self.clock_type = ClockType.SYSTEM_TIME

    def now(self):
        """Return the configured time."""
        return self.current_time


class FakeNode:
    """Provide the clock needed for exact-time lookup."""

    def __init__(self, clock):
        self.clock = clock

    def get_clock(self):
        """Return the fake clock."""
        return self.clock


class FakeTfBuffer:
    """Make a transform available only when requested by a test."""

    def __init__(self):
        self.available = False

    def lookup_transform(
        self,
        target_frame,
        tag_frame,
        observation_time,
    ):
        """Return a transform or emulate delayed TF publication."""
        if not self.available:
            raise tf2_ros.LookupException("transform not ready")

        transform = TransformStamped()
        transform.header.frame_id = target_frame
        transform.header.stamp = observation_time.to_msg()
        transform.child_frame_id = tag_frame
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        return transform


def make_detection_message(
    tag_id: int = 7,
    hamming: int = 0,
    decision_margin: float = 100.0,
) -> AprilTagDetectionArray:
    """Create a timestamped AprilTag detection array."""
    message = AprilTagDetectionArray()
    message.header.frame_id = "hand_color_image_sensor"
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 100_000_000

    detection = AprilTagDetection()
    detection.id = tag_id
    detection.hamming = hamming
    detection.decision_margin = decision_margin
    message.detections.append(detection)
    return message


def make_behavior():
    """Create a behaviour with fake ROS dependencies."""
    behavior = HandCameraTagDetection(
        max_age_sec=1.0,
        tf_pending_sec=0.5,
        max_hamming=0,
    )
    clock = FakeClock(10.2)
    behavior.node = FakeNode(clock)
    behavior.tf_buffer = FakeTfBuffer()
    return behavior, clock


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_empty_array_does_not_clear_pending_detection():
    """An empty detector frame cannot erase a valid pending frame."""
    behavior, _ = make_behavior()
    behavior._detections_callback(make_detection_message())
    behavior._detections_callback(AprilTagDetectionArray())

    assert set(behavior.pending_detections) == {7}


def test_nonzero_hamming_detection_is_rejected():
    """Corrected detections are rejected for inspection stability."""
    behavior, _ = make_behavior()
    behavior._detections_callback(
        make_detection_message(hamming=1)
    )

    assert behavior.pending_detections == {}


def test_low_decision_margin_detection_is_rejected():
    """The optional decision-margin threshold is enforced."""
    behavior = HandCameraTagDetection(
        min_decision_margin=40.0,
    )
    behavior._detections_callback(
        make_detection_message(decision_margin=20.0)
    )

    assert behavior.pending_detections == {}


def test_pending_detection_retries_until_tf_arrives():
    """A detection remains pending when its TF is published later."""
    behavior, clock = make_behavior()
    behavior._detections_callback(make_detection_message())

    first = behavior._resolve_pending_observations(
        clock.now()
    )

    assert first == {}
    assert set(behavior.pending_detections) == {7}

    behavior.tf_buffer.available = True
    second = behavior._resolve_pending_observations(
        clock.now()
    )

    assert set(second) == {7}
    assert second[7].pose.header.stamp.sec == 10
    assert behavior.pending_detections == {}


def test_pending_detection_expires():
    """An unresolved detection is eventually removed."""
    behavior, clock = make_behavior()
    behavior._detections_callback(make_detection_message())
    clock.current_time = Time(seconds=10.7)

    observations = behavior._resolve_pending_observations(
        clock.now()
    )

    assert observations == {}
    assert behavior.pending_detections == {}


def test_empty_array_does_not_clear_resolved_cache():
    """A resolved tag remains visible through a brief empty frame."""
    behavior, clock = make_behavior()
    behavior.tf_buffer.available = True
    behavior.blackboard.register_key(
        "hand_tag_observations",
        access=py_trees.common.Access.WRITE,
    )

    behavior._detections_callback(make_detection_message())
    behavior.update()
    behavior._detections_callback(AprilTagDetectionArray())
    clock.current_time = Time(seconds=10.5)
    behavior.update()

    assert set(
        behavior.blackboard.hand_tag_observations
    ) == {7}
