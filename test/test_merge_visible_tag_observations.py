"""Tests for strict base-first tag observation selection."""

import py_trees
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.behaviour_tree.nodes.sensing import (
    merge_visible_tag_observations,
)

MergeVisibleTagObservations = (
    merge_visible_tag_observations.MergeVisibleTagObservations
)


def make_tag(
    tag_id: int,
    x: float,
) -> TagElement:
    """Create a body-frame tag observation."""
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    tag.pose.pose.position.x = x
    return tag


def setup_function():
    """Clear blackboard state before each test."""
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    """Clear blackboard state after each test."""
    py_trees.blackboard.Blackboard.clear()


def create_writer():
    """Create a blackboard writer for source observations."""
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
    """Different IDs can be selected from different sources."""
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
    assert set(behavior.blackboard.visible_tags) == {1, 2}
    assert behavior.blackboard.visible_tag_sources == {
        1: "base",
        2: "hand",
    }


def test_base_always_has_precedence_for_same_id():
    """The hand observation cannot override a base observation."""
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    selected = behavior.blackboard.visible_tags[1]
    assert selected.pose.pose.position.x == 1.0
    assert behavior.blackboard.visible_tag_sources[1] == "base"


def test_hand_is_used_when_base_is_absent():
    """The hand camera supplies a tag only without a base observation."""
    writer = create_writer()
    writer.base_tag_observations = {}
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    assert behavior.blackboard.visible_tag_sources[1] == "hand"


def test_base_immediately_reclaims_returning_tag():
    """A returning base observation replaces hand fallback once."""
    writer = create_writer()
    writer.base_tag_observations = {}
    writer.hand_tag_observations = {
        1: make_tag(1, 2.0),
    }

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    assert behavior.blackboard.visible_tag_sources[1] == "hand"

    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    behavior.update()

    assert behavior.blackboard.visible_tag_sources[1] == "base"
    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == {1}
    )

    behavior.update()

    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == set()
    )


def test_empty_inputs_clear_visible_tags():
    """A tag is removed once neither source cache contains it."""
    writer = create_writer()
    writer.base_tag_observations = {
        1: make_tag(1, 1.0),
    }
    writer.hand_tag_observations = {}

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    writer.base_tag_observations = {}
    behavior.update()

    assert behavior.blackboard.visible_tags == {}
    assert behavior.blackboard.visible_tag_sources == {}
    assert (
        behavior.blackboard.visible_tag_source_changed_ids
        == {1}
    )


def test_selected_observation_is_a_deep_copy():
    """Downstream mutation cannot modify the source observation."""
    writer = create_writer()
    source_tag = make_tag(1, 1.0)
    writer.base_tag_observations = {1: source_tag}
    writer.hand_tag_observations = {}

    behavior = MergeVisibleTagObservations()
    behavior.setup()
    behavior.update()

    behavior.blackboard.visible_tags[
        1
    ].pose.pose.position.x = 99.0

    assert source_tag.pose.pose.position.x == 1.0
