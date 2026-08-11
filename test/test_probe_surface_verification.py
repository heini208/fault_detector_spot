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

    def mark_surface_verified(self, pose):
        self.verified_pose = pose

    def require_recovery(self, message):
        self.recovery_required = True
        self.recovery_message = message


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
        for stamp in (1.0, 1.1, 1.2)
    ]


def _begin(policy=None):
    refinement = _Refinement()
    coordinator = ProbeSurfaceVerificationCoordinator(policy)
    session = coordinator.begin(refinement, TEST_REQUEST_ID)
    return coordinator, session, refinement


def test_begin_requires_reached_alignment():
    refinement = _Refinement()
    refinement.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.NOT_TESTED
    )
    coordinator = ProbeSurfaceVerificationCoordinator()

    with pytest.raises(ValueError, match="aligned pre-approach"):
        coordinator.begin(refinement, TEST_REQUEST_ID)


def test_verified_samples_converge_and_capture_achieved_pose():
    coordinator, session, refinement = _begin()
    achieved_pose = object()

    decision = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.05),
        achieved_pose_object=achieved_pose,
    )

    assert decision.verified
    assert decision.correction_m == 0.0
    assert session.state == SurfaceVerificationState.CONVERGED
    assert session.measured_distance_m == pytest.approx(0.05)
    assert session.error_m == pytest.approx(0.0)
    assert refinement.verified_pose is achieved_pose


def test_correction_is_bounded_and_recovery_starts_with_motion():
    coordinator, session, refinement = _begin()

    decision = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.09),
    )

    assert not decision.verified
    assert decision.correction_m == pytest.approx(0.02)
    assert session.state == SurfaceVerificationState.MOVING
    assert not session.recovery_required

    coordinator.mark_correction_started(session, refinement)

    assert session.recovery_required
    assert refinement.recovery_required
    assert session.cumulative_correction_m == pytest.approx(0.02)


def test_cancellation_after_started_correction_requires_retraction():
    coordinator, session, refinement = _begin()
    coordinator.evaluate_samples(session, refinement, _samples(0.08))
    coordinator.mark_correction_started(session, refinement)

    coordinator.cancel(session, refinement)

    assert session.state == SurfaceVerificationState.RECOVERY_REQUIRED
    assert session.recovery_required
    assert refinement.recovery_required


def test_cumulative_limit_blocks_another_correction():
    policy = SurfaceVerificationPolicy(
        maximum_cumulative_correction_m=0.03,
    )
    coordinator, session, refinement = _begin(policy)

    first = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.08),
    )
    assert first.correction_m == pytest.approx(0.02)
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)
    coordinator.resume_sampling(session)

    second = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.08),
    )

    assert second.correction_m == 0.0
    assert session.state == SurfaceVerificationState.RECOVERY_REQUIRED
    assert session.cumulative_correction_m == pytest.approx(0.02)


def test_repeated_error_growth_stops_before_another_move():
    policy = SurfaceVerificationPolicy(
        divergence_tolerance_m=0.001,
        maximum_divergence_count=2,
    )
    coordinator, session, refinement = _begin(policy)

    coordinator.evaluate_samples(session, refinement, _samples(0.07))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)
    coordinator.resume_sampling(session)

    coordinator.evaluate_samples(session, refinement, _samples(0.08))
    coordinator.mark_correction_started(session, refinement)
    coordinator.mark_correction_succeeded(session)
    coordinator.resume_sampling(session)

    decision = coordinator.evaluate_samples(
        session,
        refinement,
        _samples(0.09),
    )

    assert decision.correction_m == 0.0
    assert session.state == SurfaceVerificationState.RECOVERY_REQUIRED
    assert "diverging" in session.detail


def test_failed_sampling_before_motion_does_not_require_retraction():
    coordinator, session, refinement = _begin()

    coordinator.mark_sampling_failed(
        session,
        refinement,
        "No fresh depth frames",
    )

    assert session.state == SurfaceVerificationState.FAILED
    assert not session.recovery_required
    assert not refinement.recovery_required
