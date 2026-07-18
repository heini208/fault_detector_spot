"""Tests for base-only authoritative tag selection."""

import py_trees
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.behaviour_tree.nodes.sensing import (
    merge_visible_tag_observations,
)

MergeVisibleTagObservations = (
    merge_visible_tag_observations.MergeVisibleTagObservations
)


def make_tag(tag_id: int, x: float) -> TagElement:
    """Create a body-frame tag observation."""
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.pose.position.x = x
    tag.pose.pose.orientation.w = 1.0
    return tag


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def create_writer():
    """Create a writer for both camera sources."""
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


def test_base_tags_are_authoritative():
    """Base observations are published unchanged by hand data."""
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
        2: make_tag(2, 3.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    assert set(behavior.blackboard.visible_tags) == {1}
    assert (
        behavior.blackboard.visible_tags[
            1
        ].pose.pose.position.x
        == 1.0
    )
    assert behavior.blackboard.visible_tag_sources == {
        1: "base",
    }


def test_hand_only_tag_is_not_promoted():
    """A hand-only tag cannot become a movement reference."""
    writer = create_writer()
    writer.base_tag_observations = {}
    writer.hand_tag_observations = {
        7: make_tag(7, 1.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    assert behavior.blackboard.visible_tags == {}
    assert behavior.blackboard.visible_tag_sources == {}


def test_source_changes_only_track_base_visibility():
    """Appearance and loss of a base tag are reported once."""
    writer = create_writer()
    writer.base_tag_observations = {
        7: make_tag(7, 1.0),
    }
    writer.hand_tag_observations = {}

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == {7}
    )

    behavior.update()

    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == set()
    )

    writer.base_tag_observations = {}
    behavior.update()

    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == {7}
    )


def test_selected_tag_is_a_deep_copy():
    """Consumers cannot mutate the source observation."""
    writer = create_writer()
    source = make_tag(7, 1.0)
    writer.base_tag_observations = {7: source}
    writer.hand_tag_observations = {}

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()
    behavior.blackboard.visible_tags[
        7
    ].pose.pose.position.x = 99.0

    assert source.pose.pose.position.x == 1.0
