"""Probe planning and guarded motion mode tests."""

import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlanner,
)


class FakeTF:

    def __init__(self, transforms):
        self.transforms = transforms

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return self.transforms[(target, source)]


def transform(parent, child, x=0.0, y=0.0, z=0.0):
    value = TransformStamped()
    value.header.frame_id = parent
    value.child_frame_id = child
    value.transform.translation.x = x
    value.transform.translation.y = y
    value.transform.translation.z = z
    value.transform.rotation.w = 1.0
    return value


def target(x=0.0, y=0.0, z=0.0, yaw=0.0):
    value = PoseStamped()
    value.header.frame_id = "body"
    value.pose.position.x = x
    value.pose.position.y = y
    value.pose.position.z = z
    value.pose.orientation.z = math.sin(yaw * 0.5)
    value.pose.orientation.w = math.cos(yaw * 0.5)
    return value


def planner():
    goals = []
    value = ProbeMotionPlanner(
        tf_listener=FakeTF({
            ("body", "hand"): transform("body", "hand"),
        }),
        speed_policy=ArmMotionSpeedPolicy(),
        build_pose_goal=lambda pose, duration: goals.append(
            (pose, duration)
        ) or object(),
    )
    value._test_goals = goals
    return value


def test_zero_motion_plan_is_a_noop():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(target(), "hand")
    )

    assert not plan.motion_required
    assert not plan.force_guard_enabled
    assert plan.goal is None
    assert value._test_goals == []


def test_rotation_only_plan_keeps_orientation_functionality():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(target(yaw=0.3), "hand")
    )

    assert plan.motion_required
    assert not plan.force_guard_enabled
    assert plan.goal is not None
    assert plan.linear_speed_mps == 0.0
    assert len(value._test_goals) == 1


def test_diagonal_translation_has_normalized_3d_direction():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(
            target(x=0.01, y=0.02, z=0.02),
            "hand",
        )
    )

    assert plan.motion_required
    assert plan.force_guard_enabled
    norm = math.sqrt(
        plan.direction_x * plan.direction_x
        + plan.direction_y * plan.direction_y
        + plan.direction_z * plan.direction_z
    )
    assert norm == pytest.approx(1.0)
    assert plan.direction_x > 0.0
    assert plan.direction_y > 0.0
    assert plan.direction_z > 0.0


def test_default_speed_is_owned_by_motion_planner():
    value = planner()

    speed = value.effective_speed(None)

    assert speed is value.speed_policy.default_speed
