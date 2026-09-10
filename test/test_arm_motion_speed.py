"""Focused tests for Cartesian arm speed calculation."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_motion_speed import (
    ArmMotionSpeed,
    ArmMotionSpeedPolicy,
)


def pose(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3],
        ),
    )


def policy():
    return ArmMotionSpeedPolicy(
        default_speed=ArmMotionSpeed(
            linear_speed_mps=0.10,
            angular_speed_rad_s=0.50,
        ),
        minimum_duration_sec=0.50,
    )


def test_default_speed_uses_current_to_target_translation():
    assert policy().duration_between(
        pose(x=0.4),
        pose(x=0.6),
    ) == pytest.approx(2.0)


def test_per_movement_speed_override_changes_duration():
    slow = ArmMotionSpeed(
        linear_speed_mps=0.05,
        angular_speed_rad_s=0.25,
    )

    assert policy().duration_between(
        pose(),
        pose(x=0.20),
        speed=slow,
    ) == pytest.approx(4.0)


def test_override_does_not_change_executor_default():
    motion_policy = policy()
    slow = ArmMotionSpeed(
        linear_speed_mps=0.05,
        angular_speed_rad_s=0.25,
    )

    assert motion_policy.duration_between(
        pose(),
        pose(x=0.20),
        speed=slow,
    ) == pytest.approx(4.0)
    assert motion_policy.duration_between(
        pose(),
        pose(x=0.20),
    ) == pytest.approx(2.0)


def test_rotation_uses_shortest_quaternion_path():
    assert policy().duration_between(
        pose(),
        pose(q=(0.0, 0.0, 1.0, 0.0)),
    ) == pytest.approx(6.283185307179586)


def test_slower_translation_or_rotation_constraint_wins():
    assert policy().duration_between(
        pose(),
        pose(
            x=0.10,
            q=(0.0, 0.0, 0.4794255386, 0.8775825620),
        ),
    ) == pytest.approx(2.0)


def test_minimum_duration_applies_to_small_motion():
    assert policy().duration_between(
        pose(),
        pose(x=0.001),
    ) == pytest.approx(0.50)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"linear_speed_mps": 0.0},
        {"angular_speed_rad_s": -1.0},
        {"linear_speed_mps": float("inf")},
    ],
)
def test_invalid_speed_values_are_rejected(kwargs):
    with pytest.raises(ValueError):
        ArmMotionSpeed(**kwargs)


def test_speed_override_requires_typed_speed():
    with pytest.raises(TypeError, match="ArmMotionSpeed"):
        policy().duration_between(
            pose(),
            pose(x=0.2),
            speed=(0.05, 0.25),
        )
