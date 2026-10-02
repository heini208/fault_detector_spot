"""Tests for routine tag-relative base-position capture geometry."""

import math
from copy import deepcopy

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.geometry.rotation import (
    multiply_quaternions,
    quaternion_from_euler,
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import PoseData
from fault_detector_spot.inspection.setup.probe_setup_motion_state_source import (
    ProbeSetupMotionStateSource,
)
from fault_detector_spot.navigation.base_motion_planner import BaseMotionPlanner
from fault_detector_spot.navigation.commands.base_to_tag_command import (
    BaseToTagCommand,
)
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose


def test_saved_base_position_round_trips_base_to_tag_offset_convention():
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = BODY_FRAME_NAME
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.position.x = 2.0
    tag.pose.pose.position.y = 1.0
    orientation = multiply_quaternions(
        quaternion_from_euler("z", math.pi / 2.0),
        quaternion_from_euler("y", -math.pi / 2.0),
    )
    tag.pose.pose.orientation.x = orientation.x
    tag.pose.pose.orientation.y = orientation.y
    tag.pose.pose.orientation.z = orientation.z
    tag.pose.pose.orientation.w = orientation.w

    source = object.__new__(ProbeSetupMotionStateSource)
    source.reference_tag = lambda _tag_id: tag
    source._lookup_pose = lambda target, source_frame, _time=None: (
        PoseData.identity()
        if target == ODOM_FRAME_NAME and source_frame == BODY_FRAME_NAME
        else _unexpected_transform(target, source_frame)
    )

    saved = source.current_base_pose_tag(7)

    assert math.isclose(saved.position.x, -1.0, abs_tol=1e-9)
    assert math.isclose(saved.position.y, 2.0, abs_tol=1e-9)
    assert saved.position.z == 0.0
    _, _, saved_yaw = quaternion_to_rpy(saved.orientation)
    assert math.isclose(saved_yaw, -math.pi / 2.0, abs_tol=1e-9)

    replay_tag = deepcopy(tag.pose)
    replay_tag.header.frame_id = ODOM_FRAME_NAME
    offset = PoseStamped()
    offset.header.frame_id = "Tag_7"
    offset.pose = pose_data_to_pose(saved)
    command = BaseToTagCommand(
        CommandID.MOVE_BASE_TO_TAG,
        Time(),
        replay_tag,
        7,
        offset=offset,
        target_frame=ODOM_FRAME_NAME,
    )
    BaseMotionPlanner._resolve_selected_tag_offset(command, 7)
    goal = command.compute_goal_pose(None)

    assert math.isclose(goal.pose.position.x, 0.0, abs_tol=1e-9)
    assert math.isclose(goal.pose.position.y, 0.0, abs_tol=1e-9)
    goal_yaw = 2.0 * math.atan2(
        goal.pose.orientation.z,
        goal.pose.orientation.w,
    )
    assert math.isclose(goal_yaw, 0.0, abs_tol=1e-9)


def _unexpected_transform(target, source):
    raise AssertionError(f"Unexpected transform request: {target} <- {source}")
