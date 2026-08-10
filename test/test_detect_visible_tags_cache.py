"""Tests for base-camera observation retention."""

import py_trees
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from fault_detector_spot.behaviour_tree.nodes.sensing import (
    detect_visible_tags,
)

DetectVisibleTags = detect_visible_tags.DetectVisibleTags


class FakeClock:
    """Provide controllable ROS time."""

    def __init__(self, seconds: float):
        self.current_time = Time(seconds=seconds)

    def now(self):
        """Return the configured time."""
        return self.current_time


class FakeNode:
    """Provide the clock needed by the behaviour."""

    def __init__(self, clock):
        self.clock = clock

    def get_clock(self):
        """Return the fake clock."""
        return self.clock


class FakeTFBuffer:
    """Return configured raw and filtered fiducial transforms."""

    def __init__(self, transforms):
        self.transforms = transforms

    def all_frames_as_yaml(self):
        return "fiducial_7:\nfiltered_fiducial_7:\n"

    def lookup_transform(self, target_frame, source_frame, lookup_time):
        return self.transforms[source_frame]


def make_transform(stamp_seconds, x):
    """Create one body-relative fiducial transform."""
    transform = TransformStamped()
    transform.header.frame_id = "body"
    transform.header.stamp.sec = stamp_seconds
    transform.child_frame_id = "fiducial"
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    return transform


def make_tag() -> TagElement:
    """Create a base-camera tag observation."""
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.orientation.w = 1.0
    return tag


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()

def register_outputs(behavior):
    behavior.blackboard.register_key(
        "base_tag_observations",
        access=py_trees.common.Access.WRITE,
    )
    behavior.blackboard.register_key(
        "visible_tags",
        access=py_trees.common.Access.WRITE,
    )

def test_missed_scan_retains_base_observation():
    """A single empty TF scan does not remove a base tag."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    clock = FakeClock(10.1)
    behavior.node = FakeNode(clock)
    register_outputs(behavior)

    observations = [{7: make_tag()}, {}]
    behavior._get_visible_tags_from_tf = (
        lambda current_time: observations.pop(0)
    )

    behavior.update()
    clock.current_time = Time(seconds=10.5)
    behavior.update()

    assert set(behavior.blackboard.base_tag_observations) == {7}
    assert set(behavior.blackboard.visible_tags) == {7}


def test_base_observation_expires():
    """A base observation disappears after its original stamp expires."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    clock = FakeClock(10.1)
    behavior.node = FakeNode(clock)
    register_outputs(behavior)

    observations = [{7: make_tag()}, {}]
    behavior._get_visible_tags_from_tf = (
        lambda current_time: observations.pop(0)
    )

    behavior.update()
    clock.current_time = Time(seconds=11.6)
    behavior.update()

    assert behavior.blackboard.base_tag_observations == {}
    assert behavior.blackboard.visible_tags == {}


def test_fresh_raw_timestamp_restores_stale_filtered_pose():
    """Raw visibility can refresh without changing filtered geometry."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    behavior.tf_buffer = FakeTFBuffer(
        {
            "fiducial_7": make_transform(20, 1.2),
            "filtered_fiducial_7": make_transform(10, 1.0),
        }
    )

    observations = behavior._get_visible_tags_from_tf(Time(seconds=20.1))

    assert set(observations) == {7}
    assert observations[7].pose.header.stamp.sec == 20
    assert observations[7].pose.pose.position.x == 1.0


def test_stale_raw_timestamp_rejects_current_filtered_pose():
    """Filtered pose availability alone does not establish visibility."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    behavior.tf_buffer = FakeTFBuffer(
        {
            "fiducial_7": make_transform(10, 1.2),
            "filtered_fiducial_7": make_transform(20, 1.0),
        }
    )

    observations = behavior._get_visible_tags_from_tf(Time(seconds=20.1))

    assert observations == {}
