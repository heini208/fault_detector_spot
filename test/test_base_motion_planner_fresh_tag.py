"""Validate semantic tag re-planning in BaseMotionPlanner."""

from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.base_motion_planner import (
    BaseMotionPlanner,
)
from fault_detector_spot.navigation.walking_profile import WalkingProfiles


class SemanticTagCommand:
    tag_id = 7
    walking_profile = "precision"

    def __init__(self):
        self.tag_pose = PoseStamped()
        self.tag_pose.header.frame_id = "odom"
        self.tag_pose.pose.orientation.w = 1.0

    def compute_goal_pose(self, _):
        return deepcopy(self.tag_pose)


def observation(x):
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(x)
    pose.pose.orientation.w = 1.0
    return SimpleNamespace(id=7, pose=pose)


def test_tag_replan_uses_new_observation_without_mutating_semantic_command():
    planner = BaseMotionPlanner(object(), WalkingProfiles())
    command = SemanticTagCommand()
    original_x = command.tag_pose.pose.position.x

    first = planner.resolve_tag_observation(
        command,
        observation(1.0),
    )
    second = planner.resolve_tag_observation(
        command,
        observation(1.2),
    )

    assert first.target.pose.position.x == 1.0
    assert second.target.pose.position.x == 1.2
    assert command.tag_pose.pose.position.x == original_x


def test_tag_replan_rejects_observation_for_another_tag():
    planner = BaseMotionPlanner(object(), WalkingProfiles())
    command = SemanticTagCommand()
    wrong = observation(1.0)
    wrong.id = 8

    try:
        planner.resolve_tag_observation(command, wrong)
    except ValueError:
        pass
    else:
        raise AssertionError("Mismatched tag observation was accepted")
