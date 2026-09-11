"""Tests for continuous speed-aware force thresholds."""

import pytest

from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)


def policy():
    return SpeedAwareForceContactPolicy(
        minimum_threshold_n=3.0,
        reference_speed_mps=0.005,
        reference_threshold_n=5.0,
        maximum_threshold_n=10.0,
        consecutive_samples=2,
    )


def test_reference_speed_reproduces_existing_close_surface_threshold():
    assert policy().threshold_for(0.005) == pytest.approx(5.0)


def test_threshold_scales_continuously_with_speed():
    contact = policy()

    assert contact.threshold_for(0.0025) == pytest.approx(4.0)
    assert contact.threshold_for(0.0100) == pytest.approx(7.0)


def test_threshold_is_capped_for_normal_arm_speed():
    assert policy().threshold_for(0.10) == pytest.approx(10.0)


def test_any_positive_speed_has_a_threshold():
    contact = policy()

    assert contact.threshold_for(0.001) > 0.0
    assert contact.threshold_for(0.037) > 0.0
    assert contact.threshold_for(0.10) > 0.0


@pytest.mark.parametrize("speed", [0.0, -0.1, float("inf")])
def test_invalid_speeds_are_rejected(speed):
    with pytest.raises(ValueError):
        policy().threshold_for(speed)
