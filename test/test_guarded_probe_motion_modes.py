"""Probe planning and guarded motion mode tests."""

import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
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
    value = ProbeMotionPlanner(
        tf_listener=FakeTF({
            ("body", "hand"): transform("body", "hand"),
        }),
        speed_policy=ArmMotionSpeedPolicy(),
    )
    return value


def test_stationary_absolute_plan_keeps_low_level_goal_semantics():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(target(), "hand")
    )

    assert plan.motion_required
    assert not plan.force_guard_enabled
    assert plan.duration_sec > 0.0
    assert not hasattr(plan, "goal")


def test_zero_relative_offset_is_detected_before_planning():
    value = planner()
    command = type("Command", (), {"offset": target()})()

    assert value.relative_command_is_noop(command)


def test_rotation_only_plan_uses_total_force_guard():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(target(yaw=0.3), "hand")
    )

    assert plan.motion_required
    assert plan.force_guard_enabled
    assert plan.duration_sec > 0.0
    assert not hasattr(plan, "goal")
    assert plan.linear_speed_mps == 0.0
    assert plan.angular_speed_rad_s > 0.0
    assert not plan.translational_motion
    assert plan.direction_x == 0.0
    assert plan.direction_y == 0.0
    assert plan.direction_z == 0.0


def test_rotation_only_plan_respects_selected_angular_speed():
    value = planner()

    plan = value.build_plan(
        lambda: value.resolved_target(target(yaw=0.5), "hand"),
        speed=ArmMotionSpeed(
            linear_speed_mps=0.08,
            angular_speed_rad_s=0.25,
        ),
    )

    assert plan.linear_speed_mps == 0.0
    assert plan.angular_speed_rad_s == pytest.approx(0.25)
    assert not plan.translational_motion


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
    assert plan.translational_motion
    assert plan.linear_speed_mps > 0.0
    assert plan.angular_speed_rad_s == 0.0
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


def test_sensor_probe_plan_reuses_attachment_geometry_without_extra_hand_tf():
    probe_frame = "hall_probe_probe"
    tf = FakeTF({
        ("body", probe_frame): transform("body", probe_frame, x=0.5),
        ("hand", probe_frame): transform("hand", probe_frame, x=0.2),
    })
    value = ProbeMotionPlanner(
        tf_listener=tf,
        speed_policy=ArmMotionSpeedPolicy(),
    )
    probe_target = target(x=0.7)

    plan = value.build_plan(
        lambda: value.resolved_target(probe_target, "hall_probe")
    )

    assert plan.motion_required
    assert plan.force_guard_enabled
    assert plan.current_hand.pose.position.x == pytest.approx(0.3)
    assert plan.target_hand.pose.position.x == pytest.approx(0.5)


def test_attached_probe_rotation_uses_total_force_guard_not_hand_arc_direction():
    probe_frame = "hall_probe_probe"
    tf = FakeTF({
        ("body", probe_frame): transform("body", probe_frame, x=0.5),
        ("hand", probe_frame): transform("hand", probe_frame, x=0.2),
    })
    value = ProbeMotionPlanner(
        tf_listener=tf,
        speed_policy=ArmMotionSpeedPolicy(),
    )

    plan = value.build_plan(
        lambda: value.resolved_target(
            target(x=0.5, yaw=0.4),
            "hall_probe",
        )
    )

    assert plan.force_guard_enabled
    assert plan.linear_speed_mps == 0.0
    assert plan.angular_speed_rad_s > 0.0
    assert not plan.translational_motion
    assert plan.direction_x == 0.0
    assert plan.direction_y == 0.0
    assert plan.direction_z == 0.0
    assert (
        plan.target_hand.pose.position.x
        != pytest.approx(plan.current_hand.pose.position.x)
        or plan.target_hand.pose.position.y
        != pytest.approx(plan.current_hand.pose.position.y)
    )
