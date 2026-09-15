"""Tests for continuous motion-aware force thresholds."""

import pytest

from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)


def policy():
    return SpeedAwareForceContactPolicy(
        minimum_threshold_n=3.0,
        reference_linear_speed_mps=0.005,
        reference_angular_speed_rad_s=0.5,
        reference_threshold_n=5.0,
        maximum_threshold_n=10.0,
        consecutive_samples=2,
    )


def test_reference_linear_speed_reproduces_reference_threshold():
    assert policy().threshold_for(0.005, 0.0) == pytest.approx(5.0)


def test_reference_angular_speed_reproduces_reference_threshold():
    assert policy().threshold_for(0.0, 0.5) == pytest.approx(5.0)


def test_threshold_scales_continuously_with_linear_speed():
    contact = policy()

    assert contact.threshold_for(0.0025, 0.0) == pytest.approx(4.0)
    assert contact.threshold_for(0.0100, 0.0) == pytest.approx(7.0)


def test_threshold_scales_continuously_with_angular_speed():
    contact = policy()

    assert contact.threshold_for(0.0, 0.25) == pytest.approx(4.0)
    assert contact.threshold_for(0.0, 1.0) == pytest.approx(7.0)


def test_combined_motion_uses_larger_normalized_speed():
    contact = policy()

    assert contact.threshold_for(0.0025, 0.5) == pytest.approx(5.0)
    assert contact.threshold_for(0.005, 0.25) == pytest.approx(5.0)


def test_threshold_is_capped_for_fast_motion():
    contact = policy()

    assert contact.threshold_for(0.10, 0.0) == pytest.approx(10.0)
    assert contact.threshold_for(0.0, 3.0) == pytest.approx(10.0)


def test_zero_motion_uses_general_minimum_threshold():
    assert policy().threshold_for(0.0, 0.0) == pytest.approx(3.0)


@pytest.mark.parametrize(
    ("linear_speed", "angular_speed"),
    [
        (-0.1, 0.0),
        (float("inf"), 0.0),
        (0.0, -0.1),
        (0.0, float("inf")),
    ],
)
def test_invalid_speeds_are_rejected(linear_speed, angular_speed):
    with pytest.raises(ValueError):
        policy().threshold_for(linear_speed, angular_speed)
