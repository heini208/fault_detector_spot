"""Tests for base-camera observation retention and frame discovery."""

import py_trees
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.sensing.behaviours import (
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
        self.frame_listing_calls = 0
        self.lookup_calls = []

    def all_frames_as_yaml(self):
        self.frame_listing_calls += 1
        return "fiducial_7:\nfiltered_fiducial_7:\n"

    def lookup_transform(self, target_frame, source_frame, lookup_time):
        self.lookup_calls.append(source_frame)
        return self.transforms[source_frame]


def make_transform(stamp_seconds, x, child_frame="fiducial"):
    """Create one body-relative fiducial transform."""
    transform = TransformStamped()
    transform.header.frame_id = "body"
    transform.header.stamp.sec = stamp_seconds
    transform.child_frame_id = child_frame
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    return transform


def make_tf_message(*child_frames):
    """Create one TF message containing named child frames."""
    message = TFMessage()
    for child_frame in child_frames:
        transform = TransformStamped()
        transform.child_frame_id = child_frame
        message.transforms.append(transform)
    return message


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


def test_runtime_frame_discovery_does_not_serialize_tf_graph():
    """Runtime discovery comes from TF messages rather than graph dumps."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    behavior.tf_buffer = FakeTFBuffer(
        {
            "fiducial_7": make_transform(20, 1.2),
            "filtered_fiducial_7": make_transform(20, 1.0),
        }
    )
    behavior._tf_frame_subscription = object()
    behavior._receive_tf_frames(
        make_tf_message(
            "fiducial_7",
            "filtered_fiducial_7",
        )
    )

    observations = behavior._get_visible_tags_from_tf(Time(seconds=20.1))

    assert set(observations) == {7}
    assert behavior.tf_buffer.frame_listing_calls == 0


def test_new_tag_frame_is_available_on_the_next_sensing_lookup():
    """A new raw TF frame is usable without a discovery polling interval."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    behavior.tf_buffer = FakeTFBuffer(
        {
            "fiducial_8": make_transform(30, 2.2),
            "filtered_fiducial_8": make_transform(30, 2.0),
        }
    )
    behavior._tf_frame_subscription = object()

    behavior._receive_tf_frames(make_tf_message("fiducial_8"))
    observations = behavior._get_visible_tags_from_tf(Time(seconds=30.1))

    assert set(observations) == {8}
    assert observations[8].pose.pose.position.x == 2.0


def test_each_tick_requeries_raw_timestamp_and_filtered_geometry():
    """Frame caching never caches pose or acquisition timestamp data."""
    behavior = DetectVisibleTags(max_age_sec=1.5)
    buffer = FakeTFBuffer(
        {
            "fiducial_7": make_transform(20, 1.2),
            "filtered_fiducial_7": make_transform(20, 1.0),
        }
    )
    behavior.tf_buffer = buffer
    behavior._tf_frame_subscription = object()
    behavior._receive_tf_frames(make_tf_message("fiducial_7"))

    first = behavior._get_visible_tags_from_tf(Time(seconds=20.1))

    buffer.transforms["fiducial_7"] = make_transform(21, 1.3)
    buffer.transforms["filtered_fiducial_7"] = make_transform(21, 1.1)
    second = behavior._get_visible_tags_from_tf(Time(seconds=21.1))

    assert first[7].pose.header.stamp.sec == 20
    assert first[7].pose.pose.position.x == 1.0
    assert second[7].pose.header.stamp.sec == 21
    assert second[7].pose.pose.position.x == 1.1
    assert buffer.lookup_calls == [
        "fiducial_7",
        "filtered_fiducial_7",
        "fiducial_7",
        "filtered_fiducial_7",
    ]


def test_filtered_frame_does_not_register_as_raw_detection():
    """Filtered geometry alone cannot discover a visible tag."""
    behavior = DetectVisibleTags()
    behavior._tf_frame_subscription = object()

    behavior._receive_tf_frames(
        make_tf_message("filtered_fiducial_7")
    )

    assert behavior._raw_fiducial_frames() == ()
