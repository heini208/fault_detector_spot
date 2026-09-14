"""Focused tests for the non-authoritative contact evidence analyzer."""

import pytest

from fault_detector_spot.manipulation.arm_contact_evidence import (
    ArmContactEvidenceAnalyzer,
    ShadowContactClassification,
)
from fault_detector_spot.manipulation.force_contact_policy import (
    SpeedAwareForceContactPolicy,
)


def analyzer():
    return ArmContactEvidenceAnalyzer(
        force_contact_policy=SpeedAwareForceContactPolicy(
            minimum_threshold_n=3.0,
            reference_speed_mps=0.005,
            reference_threshold_n=5.0,
            maximum_threshold_n=10.0,
            consecutive_samples=2,
        ),
        off_axis_speed_threshold_mps=0.04,
    )


def analyze(
    value,
    *,
    sequence=1,
    observed_at=0.1,
    force_n=11.0,
    velocity=(0.05, 0.0, 0.0),
    joint_time=None,
    joint_velocities=None,
    joint_efforts=None,
    position_error=None,
):
    return value.analyze(
        movement_sequence=sequence,
        observed_at=observed_at,
        planned_linear_speed_mps=0.10,
        movement_direction=(1.0, 0.0, 0.0),
        opposing_force_delta_n=force_n,
        hand_linear_velocity_mps=velocity,
        joint_state_received_at=joint_time,
        joint_velocities_rad_s=joint_velocities,
        joint_efforts_nm=joint_efforts,
        position_error_m=position_error,
    )


def test_sustained_force_with_low_off_axis_speed_is_external_contact():
    value = analyzer()
    value.begin_movement(1)

    first = analyze(value, observed_at=0.1, velocity=(0.05, 0.02, 0.0))
    second = analyze(value, observed_at=0.2, velocity=(0.05, 0.02, 0.0))

    assert first.classification is ShadowContactClassification.FORCE_CANDIDATE
    assert second.classification is (
        ShadowContactClassification.LIKELY_EXTERNAL_CONTACT
    )
    assert second.force_candidate_count == 2
    assert second.off_axis_hand_speed_mps == pytest.approx(0.02)
    assert second.parallel_hand_speed_mps == pytest.approx(0.05)


def test_sustained_force_with_high_off_axis_speed_is_self_motion():
    value = analyzer()
    value.begin_movement(1)

    analyze(value, observed_at=0.1, velocity=(0.05, 0.08, 0.0))
    second = analyze(value, observed_at=0.2, velocity=(0.05, 0.08, 0.0))

    assert second.classification is (
        ShadowContactClassification.LIKELY_SELF_MOTION
    )
    assert second.off_axis_hand_speed_mps == pytest.approx(0.08)
    assert second.off_axis_speed_ratio == pytest.approx(
        0.08 / (0.05 ** 2 + 0.08 ** 2) ** 0.5
    )


def test_missing_hand_velocity_never_guesses_contact_type():
    value = analyzer()
    value.begin_movement(1)

    analyze(value, observed_at=0.1, velocity=None)
    second = analyze(value, observed_at=0.2, velocity=None)

    assert second.force_candidate_count == 2
    assert second.classification is ShadowContactClassification.FORCE_CANDIDATE
    assert second.off_axis_hand_speed_mps is None


def test_force_below_threshold_resets_shadow_candidate_count():
    value = analyzer()
    value.begin_movement(1)

    analyze(value, observed_at=0.1, force_n=11.0)
    reset = analyze(value, observed_at=0.2, force_n=2.0)
    restarted = analyze(value, observed_at=0.3, force_n=11.0)

    assert reset.classification is ShadowContactClassification.NONE
    assert reset.force_candidate_count == 0
    assert restarted.classification is (
        ShadowContactClassification.FORCE_CANDIDATE
    )
    assert restarted.force_candidate_count == 1


def test_joint_and_progress_dynamics_are_logged_as_evidence_only():
    value = analyzer()
    value.begin_movement(1)

    first = analyze(
        value,
        observed_at=0.1,
        force_n=2.0,
        joint_time=1.0,
        joint_velocities=(0.1, -0.2),
        joint_efforts=(1.0, 2.0),
        position_error=0.10,
    )
    second = analyze(
        value,
        observed_at=0.2,
        force_n=2.0,
        joint_time=1.1,
        joint_velocities=(0.3, -0.6),
        joint_efforts=(2.0, 8.0),
        position_error=0.08,
    )

    assert first.joint_effort_rate_max_nm_s is None
    assert first.position_progress_rate_mps is None
    assert second.max_joint_velocity_rad_s == pytest.approx(0.6)
    assert second.joint_effort_rate_max_nm_s == pytest.approx(60.0)
    assert second.position_progress_rate_mps == pytest.approx(0.2)
    assert second.classification is ShadowContactClassification.NONE


def test_new_movement_resets_stateful_candidate_history():
    value = analyzer()
    value.begin_movement(1)
    analyze(value, sequence=1, observed_at=0.1)

    value.begin_movement(2)
    first = analyze(value, sequence=2, observed_at=0.1)

    assert first.force_candidate_count == 1
    assert first.classification is ShadowContactClassification.FORCE_CANDIDATE
