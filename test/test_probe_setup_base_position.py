"""Tests for routine tag-relative base-position capture geometry."""

import math
from collections import deque
from copy import deepcopy
from threading import RLock
from types import SimpleNamespace

import pytest

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
    requested_tag_ids = []

    def reference_tag(tag_id):
        requested_tag_ids.append(tag_id)
        return tag

    source.reference_tag = reference_tag

    current_body = PoseData.identity()
    current_body.position.x = 1.0
    current_body.position.y = 2.0

    def lookup(target, source_frame, lookup_time=None):
        assert lookup_time is None
        if target != ODOM_FRAME_NAME or source_frame != BODY_FRAME_NAME:
            return _unexpected_transform(target, source_frame)
        return current_body

    source._lookup_pose = lookup

    saved = source.current_base_pose_tag(7)

    assert requested_tag_ids == [7]
    assert math.isclose(saved.position.x, -1.0, abs_tol=1e-9)
    assert math.isclose(saved.position.y, 2.0, abs_tol=1e-9)
    assert saved.position.z == 0.0
    _, _, saved_yaw = quaternion_to_rpy(saved.orientation)
    assert math.isclose(saved_yaw, -math.pi / 2.0, abs_tol=1e-9)

    replay_tag = deepcopy(tag.pose)
    replay_tag.header.frame_id = ODOM_FRAME_NAME
    replay_tag.pose.position.x = 3.0
    replay_tag.pose.position.y = 3.0
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

    assert math.isclose(goal.pose.position.x, 1.0, abs_tol=1e-9)
    assert math.isclose(goal.pose.position.y, 2.0, abs_tol=1e-9)
    goal_yaw = 2.0 * math.atan2(
        goal.pose.orientation.z,
        goal.pose.orientation.w,
    )
    assert math.isclose(goal_yaw, 0.0, abs_tol=1e-9)


def test_base_position_capture_uses_normal_stable_tag_age_limit():
    source = object.__new__(ProbeSetupMotionStateSource)
    source._lock = RLock()
    source.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                nanoseconds=int(12.00 * 1_000_000_000)
            )
        )
    )
    history = deque()
    for stamp_seconds in (10.00, 10.10, 10.20):
        tag = TagElement()
        tag.id = 7
        tag.pose.header.frame_id = BODY_FRAME_NAME
        tag.pose.header.stamp.sec = int(stamp_seconds)
        tag.pose.header.stamp.nanosec = int(
            round(
                (stamp_seconds - int(stamp_seconds))
                * 1_000_000_000
            )
        )
        tag.pose.pose.position.x = 1.0
        tag.pose.pose.orientation.w = 1.0
        stamp = tag.pose.header.stamp
        history.append(
            (
                0.0,
                (int(stamp.sec), int(stamp.nanosec)),
                tag,
            )
        )
    source._base_tag_histories = {7: history}

    with pytest.raises(
        ValueError,
        match="Newest base-tag observation is stale",
    ):
        source.current_base_pose_tag(7)


def _unexpected_transform(target, source):
    raise AssertionError(f"Unexpected transform request: {target} <- {source}")
