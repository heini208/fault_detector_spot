"""Tests for speed-calibrated contact thresholds."""

import pytest

from fault_detector_spot.manipulation.force_contact_policy import (
    ForceThresholdCalibration,
    SpeedAwareForceContactPolicy,
    UncalibratedForceSpeed,
)


def test_single_calibration_accepts_only_its_speed():
    policy = SpeedAwareForceContactPolicy(
        [ForceThresholdCalibration(0.005, 5.0)]
    )

    assert policy.threshold_for(0.005) == pytest.approx(5.0)

    with pytest.raises(UncalibratedForceSpeed):
        policy.threshold_for(0.010)


def test_policy_interpolates_between_measured_points():
    policy = SpeedAwareForceContactPolicy([
        ForceThresholdCalibration(0.005, 5.0),
        ForceThresholdCalibration(0.015, 9.0),
    ])

    assert policy.threshold_for(0.010) == pytest.approx(7.0)


def test_policy_never_extrapolates_outside_calibrated_range():
    policy = SpeedAwareForceContactPolicy([
        ForceThresholdCalibration(0.005, 5.0),
        ForceThresholdCalibration(0.015, 9.0),
    ])

    with pytest.raises(UncalibratedForceSpeed):
        policy.threshold_for(0.002)

    with pytest.raises(UncalibratedForceSpeed):
        policy.threshold_for(0.020)
