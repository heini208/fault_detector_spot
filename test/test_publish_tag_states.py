"""Tests for separate authoritative base-camera tag publication."""

import py_trees
from fault_detector_msgs.msg import TagElement, TagElementArray

from fault_detector_spot.behaviour_tree.nodes.sensing.visible_tag_publisher import (
    PublishTagStates,
)


class FakePublisher:
    """Record messages for one topic."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    """Create topic-indexed publishers."""

    def __init__(self):
        self.publishers = {}

    def create_publisher(self, message_type, topic, qos):
        assert message_type is TagElementArray
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


def tag(tag_id):
    """Create one tag identity."""
    result = TagElement()
    result.id = tag_id
    return result


def setup_function():
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    py_trees.blackboard.Blackboard.clear()


def test_base_observations_are_not_replaced_by_other_tag_sets():
    node = FakeNode()
    behavior = PublishTagStates()
    behavior.setup(node=node)
    writer = py_trees.blackboard.Client(name="TagStateTestWriter")
    for key in (
        "visible_tags",
        "reachable_tags",
        "base_tag_observations",
    ):
        writer.register_key(key, access=py_trees.common.Access.WRITE)
    writer.visible_tags = {1: tag(1)}
    writer.reachable_tags = {2: tag(2)}
    writer.base_tag_observations = {7: tag(7)}

    assert behavior.update() == py_trees.common.Status.SUCCESS

    base_message = node.publishers[
        "fault_detector/state/base_tags"
    ].messages[-1]
    visible_message = node.publishers[
        "fault_detector/state/visible_tags"
    ].messages[-1]
    reachable_message = node.publishers[
        "fault_detector/state/reachable_tags"
    ].messages[-1]
    assert [value.id for value in base_message.elements] == [7]
    assert [value.id for value in visible_message.elements] == [1]
    assert [value.id for value in reachable_message.elements] == [2]
