"""Verify guarded probe planning uses requested/default arm speed."""

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.probe_motion_planner import (
    ProbeMotionPlanner,
)


class _TF:
    pass


def _planner():
    policy = ArmMotionSpeedPolicy(
        default_speed=ArmMotionSpeed(
            linear_speed_mps=0.10,
            angular_speed_rad_s=0.50,
        ),
        minimum_duration_sec=0.50,
    )
    return ProbeMotionPlanner(
        _TF(),
        speed_policy=policy,
        build_pose_goal=lambda target, duration: object(),
    )


def test_guard_uses_normal_default_arm_speed_when_no_override_is_given():
    planner = _planner()

    speed = planner.effective_speed(None)

    assert speed is planner.speed_policy.default_speed
    assert speed.linear_speed_mps == 0.10


def test_guard_preserves_explicit_speed_override():
    planner = _planner()
    requested = ArmMotionSpeed(
        linear_speed_mps=0.03,
        angular_speed_rad_s=0.20,
    )

    speed = planner.effective_speed(requested)

    assert speed is requested
