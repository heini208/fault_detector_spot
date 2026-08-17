"""Tests for bounded server-owned probe surface verification."""

import pytest

from fault_detector_spot.inspection.sensing.live_surface_distance import (
    SurfaceDistanceSample,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    ProbeSurfaceVerificationCoordinator,
    SurfaceVerificationPolicy,
    SurfaceVerificationState,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)


TEST_REQUEST_ID = "12345678-1234-5678-1234-567812345678"


class _Refinement:
    def __init__(self):
        self.active_stage = RefinementStage.PROBE
        self.motion_states = {
            RefinementStage.ALIGNMENT: RefinementMotionState.REACHED,
        }
        self.pending_motion = None
        self.recovery_required = False
        self.target_surface_distance_m = 0.05
        self.maximum_inward_travel_m = 0.08
        self.verified_pose = None
        self.recovery_message = ""
        self.retraction_count = 0

    def mark_surface_verified(self, pose):
        self.verified_pose = pose

    def require_recovery(self, message):
        self.recovery_required = True
        self.recovery_message = message

    def complete_retraction(self):
        self.recovery_required = False
        self.recovery_message = ""
        self.retraction_count += 1


def _samples(distance_m):
    return [
        SurfaceDistanceSample(
            distance_m=distance_m,
            stamp_seconds=stamp,
            frame_id="hand_depth",
            sample_count=20,
            valid_pixel_ratio=0.8,
            spread_m=0.001,
            source_region=ImageRegion(x=0, y=0, width=2, height=2),
        )
        for stamp in (1.0, 1.25, 1.5, 1.75, 2.0)
    ]


def _begin(policy=None):
    refinement = _Refinement()
    coordinator = ProbeSurfaceVerificationCoordinator(policy)
    session = coordinator.begin(refinement, TEST_REQUEST_ID)
    return coordinator, session, refinement


def test_default_policy_uses_slow_steps_and_multi_second_evidence():
    policy = SurfaceVerificationPolicy()

    assert policy.maximum_step_m == pytest.approx(0.01)
    assert policy.minimum_samples == 5
    assert policy.minimum_sample_span_sec == pytest.approx(1.0)


def test_surface_verification_uses_absolute_workflow_travel_guard():
    coordinator, session, refinement = _begin()

    assert refinement.maximum_inward_travel_m == pytest.approx(0.08)
    assert session.maximum_cumulative_correction_m == pytest.approx(1.0)
    assert session.maximum_cumulative_correction_m == pytest.approx(
        coordinator.policy.maximum_cumulative_correction_m
    )


def test_verified_initial_samples_converge():
    coordinator, session, refinement = _begin()
    achieved_pose = object()

    decision = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.05),
        achieved_pose_object=achieved_pose,
    )

    assert decision.verified
    assert session.state == SurfaceVerificationState.CONVERGED
    assert refinement.verified_pose is achieved_pose


def test_initial_distance_requests_only_inward_step():
    coordinator, session, refinement = _begin()

    decision = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.09),
    )

    assert not decision.verified
    assert decision.correction_m == pytest.approx(0.01)
    assert session.state == SurfaceVerificationState.MOVING


def test_kinematic_estimate_continues_without_depth():
    coordinator, session, refinement = _begin()
    coordinator.evaluate_samples(session, refinement, _samples(0.09))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)

    decision = coordinator.evaluate_estimated_distance(
        session,
        refinement,
        estimated_distance_m=0.08,
        achieved_pose_object=object(),
    )

    assert not decision.verified
    assert decision.correction_m == pytest.approx(0.01)
    assert session.state == SurfaceVerificationState.MOVING


def test_kinematic_estimate_converges():
    coordinator, session, refinement = _begin()
    coordinator.evaluate_samples(session, refinement, _samples(0.06))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)
    achieved_pose = object()

    decision = coordinator.evaluate_estimated_distance(
        session,
        refinement,
        estimated_distance_m=0.052,
        achieved_pose_object=achieved_pose,
    )

    assert decision.verified
    assert session.state == SurfaceVerificationState.CONVERGED
    assert refinement.verified_pose is achieved_pose


def test_kinematic_overshoot_requires_recovery():
    coordinator, session, refinement = _begin()
    coordinator.evaluate_samples(session, refinement, _samples(0.06))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)

    decision = coordinator.evaluate_estimated_distance(
        session,
        refinement,
        estimated_distance_m=0.04,
        achieved_pose_object=object(),
    )

    assert not decision.verified
    assert decision.correction_m == 0.0
    assert session.state == SurfaceVerificationState.RECOVERY_REQUIRED


def test_retraction_restart_clears_previous_attempt_evidence():
    coordinator, session, refinement = _begin()
    coordinator.evaluate_samples(session, refinement, _samples(0.09))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_failed(
        session,
        refinement,
        "possible contact",
    )

    coordinator.restart_after_retraction(session, refinement)

    assert session.state == SurfaceVerificationState.SAMPLING
    assert session.measured_distance_m is None
    assert session.error_m is None
    assert session.pending_correction_m is None
    assert session.cumulative_correction_m == 0.0
    assert session.iteration_count == 0
    assert session.divergence_count == 0
    assert not session.any_correction_started
    assert not session.recovery_required
    assert not refinement.recovery_required
    assert refinement.retraction_count == 1
