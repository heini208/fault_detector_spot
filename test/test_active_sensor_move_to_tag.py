"""Regression tests for active sensor geometry in tag-relative arm motion."""

import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
    OrientationModes,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)


class FakeTransformer:
    """Return fixed body and hand-to-probe transforms."""

    def __init__(self, hand_to_probe):
        self.hand_to_probe = hand_to_probe
        self.calls = []

    def lookup_a_tform_b(self, frame_a, frame_b, timeout_sec=0.0):
        self.calls.append((frame_a, frame_b))
        assert timeout_sec == 0.0
        if frame_a == "body" and frame_b == "body":
            transform = TransformStamped()
            transform.header.frame_id = "body"
            transform.child_frame_id = "body"
            transform.transform.rotation.w = 1.0
            return transform
        if frame_a == "body" and frame_b == "tag36h11:7":
            transform = TransformStamped()
            transform.header.frame_id = "body"
            transform.child_frame_id = "tag36h11:7"
            half_yaw = math.radians(25.0) * 0.5
            root_half = math.sqrt(0.5)
            transform.transform.rotation.x = (
                math.sin(half_yaw) * root_half
            )
            transform.transform.rotation.y = (
                -math.cos(half_yaw) * root_half
            )
            transform.transform.rotation.z = (
                math.sin(half_yaw) * root_half
            )
            transform.transform.rotation.w = (
                math.cos(half_yaw) * root_half
            )
            return transform
        if frame_a == "hand" and frame_b == "hall_probe_probe":
            return self.hand_to_probe
        raise AssertionError(f"Unexpected TF lookup: {frame_a} <- {frame_b}")


def target_command(sensor_id):
    tag = PoseStamped()
    tag.header.frame_id = "body"
    tag.pose.orientation.w = 1.0

    offset = PoseStamped()
    offset.header.frame_id = "body"
    offset.pose.position.x = 1.0
    offset.pose.orientation.w = 1.0

    return ManipulatorToTagCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        stamp=Time(),
        tag_pose=tag,
        tag_id=7,
        offset=offset,
        orientation_mode=OrientationModes.CUSTOM_ORIENTATION.value,
        motion_sensor_id=sensor_id,
    )


def tag_alias_command(orientation_mode):
    tag = PoseStamped()
    tag.header.frame_id = "body"
    half_yaw = math.radians(25.0) * 0.5
    root_half = math.sqrt(0.5)
    tag.pose.orientation.x = math.sin(half_yaw) * root_half
    tag.pose.orientation.y = -math.cos(half_yaw) * root_half
    tag.pose.orientation.z = math.sin(half_yaw) * root_half
    tag.pose.orientation.w = math.cos(half_yaw) * root_half

    offset = PoseStamped()
    offset.header.frame_id = "Tag_7"
    offset.pose.position.x = 0.5
    offset.pose.orientation.w = 1.0

    return ManipulatorToTagCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        stamp=Time(),
        tag_pose=tag,
        tag_id=7,
        offset=offset,
        orientation_mode=orientation_mode,
        motion_sensor_id="hand",
    )


def resolver(transformer):
    action = object.__new__(MoveCommandAction)
    action.tf_listener = transformer
    action._resolve_tag_alias = lambda frame_id: "tag36h11:7"
    action._can_transform = lambda to_frame, from_frame: True
    return action


def test_tag_move_places_active_probe_target_not_hand_at_requested_pose():
    hand_to_probe = TransformStamped()
    hand_to_probe.header.frame_id = "hand"
    hand_to_probe.child_frame_id = "hall_probe_probe"
    hand_to_probe.transform.translation.x = 0.2
    half_yaw = math.radians(90.0) * 0.5
    hand_to_probe.transform.rotation.z = math.sin(half_yaw)
    hand_to_probe.transform.rotation.w = math.cos(half_yaw)

    goal = target_command("hall_probe").compute_goal_pose(
        FakeTransformer(hand_to_probe)
    )

    assert goal.header.frame_id == "body"
    assert goal.pose.position.x == pytest.approx(1.0)
    assert goal.pose.position.y == pytest.approx(0.2)
    assert goal.pose.position.z == pytest.approx(0.0)
    assert goal.pose.orientation.x == pytest.approx(0.0)
    assert goal.pose.orientation.y == pytest.approx(0.0)
    assert goal.pose.orientation.z == pytest.approx(-math.sqrt(0.5))
    assert goal.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_no_sensor_uses_bare_hand_without_extra_probe_transform():
    transformer = FakeTransformer(TransformStamped())
    goal = target_command("hand").compute_goal_pose(transformer)

    assert ("hand", "hand") not in transformer.calls
    assert goal.pose.position.x == pytest.approx(1.0)
    assert goal.pose.position.y == pytest.approx(0.0)
    assert goal.pose.position.z == pytest.approx(0.0)
    assert goal.pose.orientation.x == pytest.approx(0.0)
    assert goal.pose.orientation.y == pytest.approx(0.0)
    assert goal.pose.orientation.z == pytest.approx(0.0)
    assert goal.pose.orientation.w == pytest.approx(1.0)


def test_relative_to_tag_keeps_orientation_offset_local_to_tag():
    transformer = FakeTransformer(TransformStamped())
    command = tag_alias_command(OrientationModes.TAG_ORIENTATION.value)

    assert resolver(transformer)._resolve_and_transform_offset_if_tag(command)

    assert command.offset.header.frame_id == "body"
    assert command.offset.pose.orientation.x == pytest.approx(0.0)
    assert command.offset.pose.orientation.y == pytest.approx(0.0)
    assert command.offset.pose.orientation.z == pytest.approx(0.0)
    assert command.offset.pose.orientation.w == pytest.approx(1.0)


def test_custom_zero_orientation_still_uses_tag_heading():
    transformer = FakeTransformer(TransformStamped())
    command = tag_alias_command(OrientationModes.CUSTOM_ORIENTATION.value)

    assert resolver(transformer)._resolve_and_transform_offset_if_tag(command)

    half_yaw = math.radians(25.0) * 0.5
    assert command.offset.pose.orientation.x == pytest.approx(0.0)
    assert command.offset.pose.orientation.y == pytest.approx(0.0)
    assert command.offset.pose.orientation.z == pytest.approx(math.sin(half_yaw))
    assert command.offset.pose.orientation.w == pytest.approx(math.cos(half_yaw))


def test_relative_to_tag_points_probe_positive_x_along_tag_negative_z():
    transformer = FakeTransformer(TransformStamped())
    command = tag_alias_command(OrientationModes.TAG_ORIENTATION.value)

    assert resolver(transformer)._resolve_and_transform_offset_if_tag(command)
    goal = command.compute_goal_pose(transformer)

    half_yaw = math.radians(25.0) * 0.5

    assert goal.pose.orientation.x == pytest.approx(0.0, abs=1e-12)
    assert goal.pose.orientation.y == pytest.approx(0.0, abs=1e-12)
    assert goal.pose.orientation.z == pytest.approx(math.sin(half_yaw))
    assert goal.pose.orientation.w == pytest.approx(math.cos(half_yaw))
