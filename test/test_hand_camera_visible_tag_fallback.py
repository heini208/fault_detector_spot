"""Regression tests for base-first hand-camera tag fallback."""

import py_trees
from fault_detector_msgs.msg import TagElement

from fault_detector_spot.sensing.behaviours.hand_camera_tag_detection import (
    HandCameraTagDetection,
)


def tag(tag_id, x):
    value = TagElement()
    value.id = tag_id
    value.pose.header.frame_id = "body"
    value.pose.pose.position.x = x
    value.pose.pose.orientation.w = 1.0
    return value


def setup_function():
    py_trees.blackboard.Blackboard.clear()


def teardown_function():
    py_trees.blackboard.Blackboard.clear()


def behavior_with_visible_tags(values):
    behavior = HandCameraTagDetection()
    behavior.blackboard.register_key(
        "visible_tags",
        access=py_trees.common.Access.WRITE,
    )
    behavior.blackboard.visible_tags = values
    return behavior


def test_hand_camera_fills_missing_visible_tag():
    behavior = behavior_with_visible_tags({})

    behavior._merge_visible_tags({7: tag(7, 2.0)})

    assert set(behavior.blackboard.visible_tags) == {7}
    assert behavior.blackboard.visible_tags[7].pose.pose.position.x == 2.0


def test_base_camera_observation_keeps_priority_for_same_tag():
    base = tag(7, 1.0)
    hand = tag(7, 2.0)
    behavior = behavior_with_visible_tags({7: base})

    behavior._merge_visible_tags({7: hand})

    assert behavior.blackboard.visible_tags[7].pose.pose.position.x == 1.0


def test_hand_fallback_adds_only_tags_missing_from_base():
    behavior = behavior_with_visible_tags({7: tag(7, 1.0)})

    behavior._merge_visible_tags(
        {
            7: tag(7, 2.0),
            8: tag(8, 3.0),
        }
    )

    assert set(behavior.blackboard.visible_tags) == {7, 8}
    assert behavior.blackboard.visible_tags[7].pose.pose.position.x == 1.0
    assert behavior.blackboard.visible_tags[8].pose.pose.position.x == 3.0
