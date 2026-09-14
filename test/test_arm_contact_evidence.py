"""Focused tests for authoritative guarded contact evidence."""

import pytest

from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
    ShadowContactClassification,
)


def analyzer():
    return ArmContactEvidenceAnalyzer(
        off_axis_speed_threshold_mps=0.04,
    )


def analyze(
    value,
    *,
    force_n=11.0,
    threshold_n=10.0,
    required=2,
    velocity=(0.05, 0.0, 0.0),
):
    return value.analyze(
        force_threshold_n=threshold_n,
        required_consecutive_samples=required,
        movement_direction=(1.0, 0.0, 0.0),
        opposing_force_delta_n=force_n,
        hand_linear_velocity_mps=velocity,
    )


def test_sustained_force_with_low_off_axis_speed_is_external_contact():
    value = analyzer()
    value.begin_movement()

    first = analyze(value, velocity=(0.05, 0.02, 0.0))
    second = analyze(value, velocity=(0.05, 0.02, 0.0))

    assert first.classification is ShadowContactClassification.FORCE_CANDIDATE
    assert second.classification is (
        ShadowContactClassification.LIKELY_EXTERNAL_CONTACT
    )
    assert second.force_candidate_count == 2
    assert second.off_axis_hand_speed_mps == pytest.approx(0.02)
    assert second.parallel_hand_speed_mps == pytest.approx(0.05)


def test_sustained_force_with_high_off_axis_speed_is_self_motion():
    value = analyzer()
    value.begin_movement()

    analyze(value, velocity=(0.05, 0.08, 0.0))
    second = analyze(value, velocity=(0.05, 0.08, 0.0))

    assert second.classification is (
        ShadowContactClassification.LIKELY_SELF_MOTION
    )
    assert second.off_axis_hand_speed_mps == pytest.approx(0.08)
    assert second.off_axis_speed_ratio == pytest.approx(
        0.08 / (0.05 ** 2 + 0.08 ** 2) ** 0.5
    )


def test_missing_hand_velocity_never_guesses_contact_type():
    value = analyzer()
    value.begin_movement()

    analyze(value, velocity=None)
    second = analyze(value, velocity=None)

    assert second.force_candidate_count == 2
    assert second.classification is ShadowContactClassification.FORCE_CANDIDATE
    assert second.off_axis_hand_speed_mps is None


def test_force_below_authoritative_threshold_resets_candidate_count():
    value = analyzer()
    value.begin_movement()

    analyze(value, force_n=11.0)
    reset = analyze(value, force_n=9.0)
    restarted = analyze(value, force_n=11.0)

    assert reset.classification is ShadowContactClassification.NONE
    assert reset.force_candidate_count == 0
    assert restarted.classification is (
        ShadowContactClassification.FORCE_CANDIDATE
    )
    assert restarted.force_candidate_count == 1


def test_explicit_authoritative_threshold_is_used_without_policy_copy():
    value = analyzer()
    value.begin_movement()

    below = analyze(value, force_n=6.0, threshold_n=7.0)
    above = analyze(value, force_n=6.0, threshold_n=5.0)

    assert not below.force_threshold_exceeded
    assert above.force_threshold_exceeded
    assert above.force_threshold_n == pytest.approx(5.0)


def test_new_movement_resets_candidate_history():
    value = analyzer()
    value.begin_movement()
    analyze(value)

    value.begin_movement()
    first = analyze(value)

    assert first.force_candidate_count == 1
    assert first.classification is ShadowContactClassification.FORCE_CANDIDATE
