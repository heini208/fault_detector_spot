"""Verify guarded movement uses the requested/default arm speed."""

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
    ArmMotionSpeedPolicy,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)


class _TF:
    pass


def _executor():
    policy = ArmMotionSpeedPolicy(
        default_speed=ArmMotionSpeed(
            linear_speed_mps=0.10,
            angular_speed_rad_s=0.50,
        ),
        minimum_duration_sec=0.50,
    )
    return ArmMovementExecutor(
        _TF(),
        speed_policy=policy,
    )


def test_guard_uses_normal_default_arm_speed_when_no_override_is_given():
    executor = _executor()

    speed = executor._effective_guarded_speed(None)

    assert speed is executor.speed_policy.default_speed
    assert speed.linear_speed_mps == 0.10


def test_guard_preserves_explicit_speed_override():
    executor = _executor()
    requested = ArmMotionSpeed(
        linear_speed_mps=0.03,
        angular_speed_rad_s=0.20,
    )

    speed = executor._effective_guarded_speed(requested)

    assert speed is requested
