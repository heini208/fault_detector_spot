"""Focused tests for base endpoint correction policy."""

import pytest

from fault_detector_spot.navigation.base_correction_policy import (
    BaseCorrectionConfig,
    BaseCorrectionDecision,
    BaseCorrectionPolicy,
)


def test_first_out_of_tolerance_error_requests_correction():
    policy = BaseCorrectionPolicy()
    result = policy.decide(
        (0.2, 0.0),
        position_tolerance_m=0.1,
        yaw_tolerance_rad=0.1,
    )
    assert (
        result.decision
        is BaseCorrectionDecision.CORRECT
    )
    assert result.attempt == 1


def test_inside_tolerance_does_not_retry():
    policy = BaseCorrectionPolicy()
    result = policy.decide(
        (0.05, 0.05),
        position_tolerance_m=0.1,
        yaw_tolerance_rad=0.1,
    )
    assert result.decision is BaseCorrectionDecision.FAIL
    assert policy.attempts == 0


def test_progress_requirement_is_owned_by_policy():
    policy = BaseCorrectionPolicy(
        BaseCorrectionConfig(minimum_progress_ratio=0.10)
    )
    assert (
        policy.decide((0.2, 0.0), 0.1, 0.1).decision
        is BaseCorrectionDecision.CORRECT
    )
    result = policy.decide((0.19, 0.0), 0.1, 0.1)
    assert result.decision is BaseCorrectionDecision.FAIL
    assert "insufficient progress" in result.detail


def test_attempt_limit_is_owned_by_policy():
    policy = BaseCorrectionPolicy(
        BaseCorrectionConfig(maximum_attempts=1)
    )
    assert (
        policy.decide((0.2, 0.0), 0.1, 0.1).decision
        is BaseCorrectionDecision.CORRECT
    )
    result = policy.decide((0.1 + 1e-6, 0.0), 0.1, 0.1)
    assert result.decision is BaseCorrectionDecision.FAIL
    assert "attempt limit" in result.detail


def test_reset_clears_attempt_history():
    policy = BaseCorrectionPolicy()
    policy.decide((0.2, 0.0), 0.1, 0.1)
    assert policy.attempts == 1
    policy.reset()
    assert policy.attempts == 0
    assert (
        policy.decide((0.2, 0.0), 0.1, 0.1).decision
        is BaseCorrectionDecision.CORRECT
    )


@pytest.mark.parametrize("value", [-1, 1.5, True])
def test_invalid_attempt_limit_rejected(value):
    with pytest.raises(ValueError):
        BaseCorrectionConfig(maximum_attempts=value)


@pytest.mark.parametrize("value", [0.0, 1.0, -0.1])
def test_invalid_progress_ratio_rejected(value):
    with pytest.raises(ValueError):
        BaseCorrectionConfig(minimum_progress_ratio=value)
