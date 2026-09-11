"""Guarded probe motion-mode and threshold override tests."""

from copy import deepcopy
import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStowState
from fault_detector_spot.manipulation.guarded_probe_execution import (
    GuardedProbeExecution,
    GuardedProbePlan,
)


class FakePolicy:
    consecutive_samples = 2

    def __init__(self):
        self.speeds = []

    def threshold_for(self, speed):
        self.speeds.append(speed)
        return 12.0


class FakeArmState:
    last_received_at = 0.0

    def stow_state(self):
        return ArmStowState.DEPLOYED

    def is_stale(self):
        return False


class FakeTF:

    def __init__(self, hand_transform):
        self.hand_transform = hand_transform

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        assert source == executor_module.HAND_FRAME_NAME
        return deepcopy(self.hand_transform)


def transform(parent, child):
    value = TransformStamped()
    value.header.frame_id = parent
    value.child_frame_id = child
    value.transform.rotation.w = 1.0
    return value


def target(yaw=0.0):
    value = PoseStamped()
    value.header.frame_id = executor_module.GRAV_ALIGNED_BODY_FRAME_NAME
    value.pose.orientation.z = math.sin(yaw * 0.5)
    value.pose.orientation.w = math.cos(yaw * 0.5)
    return value


def executor(monkeypatch):
    tf = FakeTF(
        transform(
            executor_module.GRAV_ALIGNED_BODY_FRAME_NAME,
            executor_module.HAND_FRAME_NAME,
        )
    )
    policy = FakePolicy()
    value = ArmMovementExecutor(
        tf,
        arm_state_source=FakeArmState(),
        force_contact_policy=policy,
    )
    monkeypatch.setattr(
        value,
        "_build_pose_goal",
        lambda target_pose, duration: ("goal", duration),
    )
    return value, policy


def test_zero_motion_plan_is_a_successful_noop(monkeypatch):
    value, policy = executor(monkeypatch)

    plan = value._build_guarded_probe_plan(
        lambda: (target(), "hand"),
    )

    assert not plan.motion_required
    assert not plan.force_guard_enabled
    assert plan.goal is None
    assert policy.speeds == []


def test_rotation_only_plan_keeps_orientation_functionality(monkeypatch):
    value, policy = executor(monkeypatch)

    plan = value._build_guarded_probe_plan(
        lambda: (target(yaw=0.3), "hand"),
    )

    assert plan.motion_required
    assert not plan.force_guard_enabled
    assert plan.goal is not None
    assert plan.linear_speed_mps == 0.0
    assert policy.speeds == []


def test_translation_can_override_force_threshold(monkeypatch):
    value, policy = executor(monkeypatch)
    pose = target()
    pose.pose.position.x = 0.01

    plan = value._build_guarded_probe_plan(
        lambda: (pose, "hand"),
        force_threshold_n=5.0,
    )

    assert plan.force_guard_enabled
    assert plan.force_threshold_n == pytest.approx(5.0)
    assert policy.speeds == []


def test_translation_without_override_uses_generic_policy(monkeypatch):
    value, policy = executor(monkeypatch)
    pose = target()
    pose.pose.position.x = 0.01

    plan = value._build_guarded_probe_plan(
        lambda: (pose, "hand"),
    )

    assert plan.force_guard_enabled
    assert plan.force_threshold_n == pytest.approx(12.0)
    assert len(policy.speeds) == 1


class NoopSettling:
    def reset(self):
        pass


class NoopBaseline:
    def reset(self):
        pass


def guard(start_goal):
    return GuardedProbeExecution(
        arm_state_source=object(),
        settling_detector=NoopSettling(),
        force_baseline_sampler=NoopBaseline(),
        start_goal=start_goal,
        poll_goal=lambda: ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Succeeded",
        ),
        cancel_goal=lambda: None,
        current_hand_pose=lambda: target(),
        build_motion_goal=lambda current, target_pose, speed: object(),
        default_angular_speed_rad_s=0.5,
        force_stale_timeout_sec=0.25,
        retreat_distance_m=0.01,
        retreat_speed_mps=0.01,
    )


def noop_plan():
    pose = target()
    return GuardedProbePlan(
        goal=None,
        current_hand=pose,
        target_hand=pose,
        direction_x=0.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.0,
        force_threshold_n=None,
        contact_consecutive_samples=2,
        motion_required=False,
        force_guard_enabled=False,
    )


def rotation_plan():
    pose = target()
    return GuardedProbePlan(
        goal=object(),
        current_hand=pose,
        target_hand=pose,
        direction_x=0.0,
        direction_y=0.0,
        direction_z=0.0,
        linear_speed_mps=0.0,
        force_threshold_n=None,
        contact_consecutive_samples=2,
        motion_required=True,
        force_guard_enabled=False,
    )


def test_noop_finishes_without_sending_goal():
    goals = []
    value = guard(
        lambda goal: goals.append(goal)
        or ArmMovementUpdate(ArmMovementOutcome.RUNNING, "sent")
    )

    update = value.start(noop_plan)

    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert "Skipped zero arm movement" in update.detail
    assert goals == []


def test_rotation_only_starts_goal_without_force_baseline():
    goals = []
    value = guard(
        lambda goal: goals.append(goal)
        or ArmMovementUpdate(ArmMovementOutcome.RUNNING, "sent")
    )

    update = value.start(rotation_plan)

    assert update.outcome is ArmMovementOutcome.RUNNING
    assert len(goals) == 1


class RelativeCommand:
    def __init__(self, offset):
        self.offset = offset


def test_relative_identity_offset_is_skipped_before_guard_setup(monkeypatch):
    value, _ = executor(monkeypatch)
    offset = target()
    command = RelativeCommand(offset)

    update = value.relative(command)

    assert update.outcome is ArmMovementOutcome.SUCCESS
    assert "Skipped zero arm movement" in update.detail


def test_relative_rotation_only_is_not_mistaken_for_noop(monkeypatch):
    value, _ = executor(monkeypatch)
    offset = target(yaw=0.3)
    command = RelativeCommand(offset)

    assert not value._relative_command_is_noop(command)
