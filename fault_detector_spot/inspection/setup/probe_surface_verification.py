"""Coordinate bounded server-owned probe surface verification."""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    SurfaceDistanceAggregate,
    aggregate_surface_distance_samples,
)
from fault_detector_spot.inspection.sensing.surface_distance_validation import (
    require_positive_finite_distance,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)


class SurfaceVerificationState(str, Enum):
    """Server-owned states for closed-loop surface verification."""

    IDLE = "idle"
    SAMPLING = "sampling"
    MOVING = "moving"
    SETTLING = "settling"
    CONVERGED = "converged"
    FAILED = "failed"
    CANCELLED = "cancelled"
    RECOVERY_REQUIRED = "recovery_required"


@dataclass(frozen=True)
class SurfaceVerificationPolicy:
    """Safety limits for one surface-verification workflow."""

    tolerance_m: float = 0.005
    maximum_step_m: float = 0.010
    maximum_cumulative_correction_m: float = 1.000
    maximum_iterations: int = 40
    minimum_samples: int = 5
    minimum_sample_span_sec: float = 1.0
    stability_tolerance_m: float = 0.005
    divergence_tolerance_m: float = 0.005
    maximum_divergence_count: int = 2

    def validate(self) -> None:
        for label, value in (
            ("Surface-distance tolerance", self.tolerance_m),
            ("Maximum correction step", self.maximum_step_m),
            (
                "Maximum cumulative correction",
                self.maximum_cumulative_correction_m,
            ),
            ("Minimum sampling span", self.minimum_sample_span_sec),
            ("Distance stability tolerance", self.stability_tolerance_m),
            ("Divergence tolerance", self.divergence_tolerance_m),
        ):
            require_positive_finite_distance(value, label)
        for label, value, minimum in (
            ("Maximum verification iterations", self.maximum_iterations, 1),
            ("Minimum surface samples", self.minimum_samples, 3),
            (
                "Maximum divergence count",
                self.maximum_divergence_count,
                1,
            ),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < minimum
            ):
                raise ValueError(
                    f"{label} must be an integer greater than or equal to "
                    f"{minimum}"
                )


@dataclass(frozen=True)
class SurfaceVerificationDecision:
    """Result of evaluating one stable post-settle sample window."""

    verified: bool
    correction_m: float
    resample_required: bool
    aggregate: SurfaceDistanceAggregate


@dataclass(frozen=True)
class SurfaceKinematicDecision:
    """Result of evaluating one post-motion kinematic distance estimate."""

    verified: bool
    correction_m: float
    estimated_distance_m: float


@dataclass
class ProbeSurfaceVerificationSession:
    """Mutable authoritative state for one verification request."""

    request_id: str
    target_distance_m: float
    maximum_cumulative_correction_m: float
    policy: SurfaceVerificationPolicy
    state: SurfaceVerificationState = SurfaceVerificationState.SAMPLING
    measured_distance_m: Optional[float] = None
    error_m: Optional[float] = None
    last_correction_m: Optional[float] = None
    pending_correction_m: Optional[float] = None
    cumulative_correction_m: float = 0.0
    iteration_count: int = 0
    divergence_count: int = 0
    previous_error_m: Optional[float] = None
    correction_started: bool = False
    any_correction_started: bool = False
    recovery_required: bool = False
    detail: str = ""

    @property
    def active(self) -> bool:
        return self.state in {
            SurfaceVerificationState.SAMPLING,
            SurfaceVerificationState.MOVING,
            SurfaceVerificationState.SETTLING,
        }


class ProbeSurfaceVerificationCoordinator:
    """Own bounded distance decisions independently of ROS transport."""

    def __init__(
        self,
        policy: Optional[SurfaceVerificationPolicy] = None,
    ):
        self.policy = policy or SurfaceVerificationPolicy()
        self.policy.validate()

    def begin(
        self,
        refinement,
        request_id: str,
    ) -> ProbeSurfaceVerificationSession:
        request_id = validate_request_id(request_id)
        self._require_ready_refinement(refinement)
        target_distance_m = float(refinement.target_surface_distance_m)
        require_positive_finite_distance(
            target_distance_m,
            "Target surface distance",
        )
        refinement_limit = float(refinement.maximum_inward_travel_m)
        require_positive_finite_distance(
            refinement_limit,
            "Maximum refinement travel",
        )
        limit = min(
            refinement_limit,
            self.policy.maximum_cumulative_correction_m,
        )
        return ProbeSurfaceVerificationSession(
            request_id=request_id,
            target_distance_m=target_distance_m,
            maximum_cumulative_correction_m=limit,
            policy=self.policy,
            detail="Collecting fresh post-settle surface-distance samples",
        )

    def evaluate_samples(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
        samples,
        achieved_pose_object=None,
    ) -> SurfaceVerificationDecision:
        self._require_state(
            session,
            SurfaceVerificationState.SAMPLING,
        )
        previous_error = session.error_m
        aggregate = aggregate_surface_distance_samples(
            samples,
            session.target_distance_m,
            session.policy.maximum_step_m,
            tolerance_m=session.policy.tolerance_m,
            minimum_samples=session.policy.minimum_samples,
            minimum_span_sec=session.policy.minimum_sample_span_sec,
            stability_tolerance_m=session.policy.stability_tolerance_m,
        )
        session.iteration_count += 1
        session.measured_distance_m = aggregate.distance_m
        session.error_m = aggregate.correction.error_m
        self._update_divergence(session, previous_error)

        if aggregate.verified:
            self._mark_converged(
                session,
                refinement,
                achieved_pose_object,
                aggregate.distance_m,
            )
            return SurfaceVerificationDecision(
                verified=True,
                correction_m=0.0,
                resample_required=False,
                aggregate=aggregate,
            )

        if self._terminal_error(session, refinement):
            return SurfaceVerificationDecision(
                verified=False,
                correction_m=0.0,
                resample_required=False,
                aggregate=aggregate,
            )

        correction_m = aggregate.correction.inward_correction_m
        if correction_m < -session.policy.tolerance_m:
            self._fail(
                session,
                refinement,
                "Close-to-surface execution does not move away from the surface",
            )
            correction_m = 0.0
        elif math.isclose(correction_m, 0.0, abs_tol=1e-12):
            session.previous_error_m = session.error_m
            session.detail = (
                "Stable samples are not jointly inside tolerance; resample"
            )
            return SurfaceVerificationDecision(
                verified=False,
                correction_m=0.0,
                resample_required=True,
                aggregate=aggregate,
            )
        else:
            self._request_correction(
                session,
                refinement,
                correction_m,
            )

        return SurfaceVerificationDecision(
            verified=False,
            correction_m=correction_m,
            resample_required=False,
            aggregate=aggregate,
        )

    def evaluate_estimated_distance(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
        estimated_distance_m: float,
        achieved_pose_object,
    ) -> SurfaceKinematicDecision:
        """Choose the next inward step from frozen-surface kinematics."""
        self._require_state(
            session,
            SurfaceVerificationState.SETTLING,
        )
        require_positive_finite_distance(
            estimated_distance_m,
            "Estimated surface distance",
        )
        previous_error = session.error_m
        estimated_distance_m = float(estimated_distance_m)
        error_m = estimated_distance_m - session.target_distance_m
        session.iteration_count += 1
        session.measured_distance_m = estimated_distance_m
        session.error_m = error_m
        self._update_divergence(session, previous_error)

        if abs(error_m) <= session.policy.tolerance_m:
            self._mark_converged(
                session,
                refinement,
                achieved_pose_object,
                estimated_distance_m,
            )
            return SurfaceKinematicDecision(
                verified=True,
                correction_m=0.0,
                estimated_distance_m=estimated_distance_m,
            )

        if error_m < -session.policy.tolerance_m:
            self._fail(
                session,
                refinement,
                "Probe passed the requested surface stand-off",
            )
            return SurfaceKinematicDecision(
                verified=False,
                correction_m=0.0,
                estimated_distance_m=estimated_distance_m,
            )

        if self._terminal_error(session, refinement):
            return SurfaceKinematicDecision(
                verified=False,
                correction_m=0.0,
                estimated_distance_m=estimated_distance_m,
            )

        correction_m = min(
            session.policy.maximum_step_m,
            error_m,
        )
        self._request_correction(
            session,
            refinement,
            correction_m,
        )
        return SurfaceKinematicDecision(
            verified=False,
            correction_m=correction_m,
            estimated_distance_m=estimated_distance_m,
        )

    def mark_correction_started(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
    ) -> None:
        self._require_state(
            session,
            SurfaceVerificationState.MOVING,
        )
        correction_m = session.pending_correction_m
        if correction_m is None:
            raise RuntimeError("No surface correction is pending")
        if session.correction_started:
            raise RuntimeError("Surface correction is already marked started")
        session.correction_started = True
        session.any_correction_started = True
        session.cumulative_correction_m += abs(correction_m)
        session.recovery_required = True
        refinement.require_recovery(
            "Probe moved away from the aligned pre-approach pose"
        )

    def mark_correction_succeeded(
        self,
        session: ProbeSurfaceVerificationSession,
    ) -> None:
        self._require_state(
            session,
            SurfaceVerificationState.MOVING,
        )
        if not session.correction_started:
            raise RuntimeError("Surface correction did not start")
        session.pending_correction_m = None
        session.correction_started = False
        session.state = SurfaceVerificationState.SETTLING
        session.detail = "Correction reached; wait for the arm to settle"

    def resume_sampling(
        self,
        session: ProbeSurfaceVerificationSession,
    ) -> None:
        self._require_state(
            session,
            SurfaceVerificationState.SETTLING,
        )
        session.state = SurfaceVerificationState.SAMPLING
        session.detail = "Collecting fresh post-settle samples"

    def mark_correction_failed(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
        detail: str,
    ) -> None:
        self._require_state(
            session,
            SurfaceVerificationState.MOVING,
        )
        self._fail(session, refinement, detail)

    def mark_sampling_failed(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
        detail: str,
    ) -> None:
        self._require_state(
            session,
            SurfaceVerificationState.SAMPLING,
        )
        self._fail(session, refinement, detail)

    def abort(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
        detail: str,
    ) -> None:
        if not session.active:
            return
        self._fail(session, refinement, detail)

    def cancel(
        self,
        session: ProbeSurfaceVerificationSession,
        refinement,
    ) -> None:
        if not session.active:
            return
        session.pending_correction_m = None
        session.correction_started = False
        if (
            session.any_correction_started
            or bool(getattr(refinement, "recovery_required", False))
        ):
            session.recovery_required = True
            session.state = SurfaceVerificationState.RECOVERY_REQUIRED
            session.detail = "Surface verification cancelled; retract required"
            refinement.require_recovery(session.detail)
            return
        session.state = SurfaceVerificationState.CANCELLED
        session.detail = "Surface verification cancelled"

    def _request_correction(
        self,
        session,
        refinement,
        correction_m,
    ) -> None:
        projected_travel = (
            session.cumulative_correction_m + abs(correction_m)
        )
        if (
            projected_travel
            > session.maximum_cumulative_correction_m + 1e-12
        ):
            self._fail(
                session,
                refinement,
                "Surface correction exceeds the cumulative travel limit",
            )
            return
        session.previous_error_m = session.error_m
        session.last_correction_m = correction_m
        session.pending_correction_m = correction_m
        session.correction_started = False
        session.state = SurfaceVerificationState.MOVING
        session.detail = (
            f"Request one bounded axial correction of {correction_m:+.4f} m"
        )

    def _terminal_error(self, session, refinement) -> bool:
        if (
            session.divergence_count
            >= session.policy.maximum_divergence_count
        ):
            self._fail(
                session,
                refinement,
                "Surface-distance error is diverging",
            )
            return True
        if session.iteration_count >= session.policy.maximum_iterations:
            self._fail(
                session,
                refinement,
                "Surface verification exceeded the iteration limit",
            )
            return True
        return False

    @staticmethod
    def _mark_converged(
        session,
        refinement,
        achieved_pose_object,
        measured_distance_m,
    ) -> None:
        if achieved_pose_object is None:
            raise ValueError(
                "Verified surface distance requires an achieved probe pose"
            )
        refinement.mark_surface_verified(achieved_pose_object)
        session.state = SurfaceVerificationState.CONVERGED
        session.measured_distance_m = float(measured_distance_m)
        session.error_m = (
            float(measured_distance_m) - session.target_distance_m
        )
        session.pending_correction_m = None
        session.correction_started = False
        session.detail = (
            "Surface distance converged within the configured tolerance"
        )

    @staticmethod
    def _require_ready_refinement(refinement) -> None:
        if refinement is None:
            raise ValueError("Probe refinement is not active")
        if refinement.active_stage != RefinementStage.PROBE:
            raise ValueError("Open the probe refinement stage first")
        if (
            refinement.motion_states[RefinementStage.ALIGNMENT]
            != RefinementMotionState.REACHED
        ):
            raise ValueError(
                "Reach the aligned pre-approach before surface verification"
            )
        if refinement.pending_motion is not None:
            raise RuntimeError("A refinement movement is already active")
        if refinement.recovery_required:
            raise RuntimeError("Retract before starting surface verification")

    @staticmethod
    def _require_state(session, expected) -> None:
        if not isinstance(session, ProbeSurfaceVerificationSession):
            raise TypeError("Expected a ProbeSurfaceVerificationSession")
        if session.state != expected:
            raise RuntimeError(
                f"Surface verification is {session.state.value}, "
                f"expected {expected.value}"
            )

    @staticmethod
    def _update_divergence(session, previous_error) -> None:
        if previous_error is None:
            session.divergence_count = 0
            return
        if (
            abs(session.error_m)
            > abs(previous_error)
            + session.policy.divergence_tolerance_m
        ):
            session.divergence_count += 1
        else:
            session.divergence_count = 0

    @staticmethod
    def _fail(session, refinement, detail) -> None:
        detail = str(detail).strip() or "Surface verification failed"
        session.pending_correction_m = None
        session.correction_started = False
        requires_recovery = (
            session.any_correction_started
            or bool(getattr(refinement, "recovery_required", False))
        )
        if requires_recovery:
            session.recovery_required = True
            session.state = SurfaceVerificationState.RECOVERY_REQUIRED
            refinement.require_recovery(detail)
        else:
            session.state = SurfaceVerificationState.FAILED
        session.detail = detail


__all__ = [
    "ProbeSurfaceVerificationCoordinator",
    "ProbeSurfaceVerificationSession",
    "SurfaceKinematicDecision",
    "SurfaceVerificationDecision",
    "SurfaceVerificationPolicy",
    "SurfaceVerificationState",
]
