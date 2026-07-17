import py_trees
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.behaviour_tree.nodes.sensing.merge_visible_tag_observations import (
    MergeVisibleTagObservations,
)


def make_tag(
    tag_id: int,
    x: float,
) -> TagElement:
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    tag.pose.pose.position.x = x
    return tag


def setup_function():
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    py_trees.blackboard.Blackboard.clear()


def create_writer():
    writer = py_trees.blackboard.Client(
        name="TagMergeTestWriter"
    )
    writer.register_key(
        "base_tag_observations",
        access=py_trees.common.Access.WRITE,
    )
    writer.register_key(
        "hand_tag_observations",
        access=py_trees.common.Access.WRITE,
    )
    return writer


def test_merges_different_tag_ids():
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {
        2: make_tag(2, 2.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()

    status = behavior.update()

    assert status == py_trees.common.Status.SUCCESS
    assert set(
        behavior.blackboard.visible_tags.keys()
    ) == {1, 2}


def test_base_source_has_default_precedence():
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
    }

    behavior = MergeVisibleTagObservations(
        preferred_source="base"
    )
    behavior.setup()
    behavior.update()

    assert (
        behavior.blackboard.visible_tags[
            1
        ].pose.pose.position.x
        == 1.0
    )


def test_hand_source_can_be_preferred():
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
    }

    behavior = MergeVisibleTagObservations(
        preferred_source="hand"
    )
    behavior.setup()
    behavior.update()

    assert (
        behavior.blackboard.visible_tags[
            1
        ].pose.pose.position.x
        == 2.0
    )


def test_empty_inputs_clear_visible_tags():
    writer = create_writer()
    writer.base_tag_observations = {}
    writer.hand_tag_observations = {}

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.blackboard.visible_tags = {
        1: make_tag(1, 1.0),
    }

    behavior.update()

    assert behavior.blackboard.visible_tags == {}