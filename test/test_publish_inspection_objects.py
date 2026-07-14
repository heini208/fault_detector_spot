"""Tests for inspection object state publishing."""

import py_trees
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from fault_detector_msgs.msg import (
    InspectionObjectState,
    InspectionObjectStateArray,
)
from fault_detector_spot.behaviour_tree.nodes.sensing.publish_inspection_objects import (
    PublishInspectionObjects,
)
from fault_detector_spot.inspection.object_pose_resolver import (
    ObjectPoseState,
    ResolvedObjectPose,
)


class FakePublisher:
    """Store published messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Store a published message."""
        self.messages.append(message)


class FakeNow:
    """Provide a ROS time message."""

    def to_msg(self):
        """Return a fixed timestamp."""
        return Time(sec=12, nanosec=34)


class FakeClock:
    """Provide a fixed current time."""

    def now(self):
        """Return the fake current time."""
        return FakeNow()


class FakeNode:
    """Minimal ROS node for publisher tests."""

    def __init__(self):
        self.publisher = FakePublisher()
        self.topic_name = None
        self.message_type = None
        self.qos = None

    def create_publisher(
        self,
        message_type,
        topic_name,
        qos,
    ):
        """Create and record a fake publisher."""
        self.message_type = message_type
        self.topic_name = topic_name
        self.qos = qos
        return self.publisher

    def get_clock(self):
        """Return a fixed clock."""
        return FakeClock()


def create_writer():
    """Create a blackboard writer."""
    writer = py_trees.blackboard.Client(
        name="ObjectPublisherTestWriter"
    )

    writer.register_key(
        "inspection_objects",
        access=py_trees.common.Access.WRITE,
    )
    writer.register_key(
        "inspection_object_error",
        access=py_trees.common.Access.WRITE,
    )

    return writer


def create_pose(x):
    """Create a valid map-frame pose."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def test_publisher_publishes_resolved_objects():
    """Resolved objects are published to the state topic."""
    writer = create_writer()

    remembered_pose = create_pose(1.0)

    writer.inspection_objects = {
        "motor_a": ResolvedObjectPose(
            object_id="motor_a",
            tag_id=23,
            state=ObjectPoseState.REMEMBERED,
            selected_pose=remembered_pose,
            remembered_pose=remembered_pose,
            message="Using remembered landmark pose",
        )
    }
    writer.inspection_object_error = ""

    node = FakeNode()
    behavior = PublishInspectionObjects()
    behavior.setup(node=node)

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert (
        node.message_type
        is InspectionObjectStateArray
    )
    assert (
        node.topic_name
        == "fault_detector/state/inspection_objects"
    )
    assert len(node.publisher.messages) == 1

    message = node.publisher.messages[0]

    assert message.header.frame_id == "map"
    assert message.header.stamp.sec == 12
    assert len(message.objects) == 1

    object_message = message.objects[0]

    assert object_message.object_id == "motor_a"
    assert object_message.tag_id == 23
    assert (
        object_message.state
        == InspectionObjectState.REMEMBERED
    )
    assert object_message.has_remembered_pose


def test_publisher_publishes_empty_error_state():
    """Empty object states still publish resolver errors."""
    writer = create_writer()
    writer.inspection_objects = {}
    writer.inspection_object_error = (
        "Map metadata does not exist"
    )

    node = FakeNode()
    behavior = PublishInspectionObjects()
    behavior.setup(node=node)

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert len(node.publisher.messages) == 1

    message = node.publisher.messages[0]

    assert message.objects == []
    assert (
        message.error_message
        == "Map metadata does not exist"
    )