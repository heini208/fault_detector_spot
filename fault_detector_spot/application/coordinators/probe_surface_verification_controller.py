"""Surface-verification state transitions for probe setup."""

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_surface_verification import (
    ProbeSurfaceVerificationCoordinator,
    SurfaceVerificationState,
)


class ProbeSurfaceVerificationController:
    """Own one probe draft's closed-loop verification state transitions."""

    def __init__(self, refinement_controller, state_lock):
        self.refinement_controller = refinement_controller
        self.state_lock = state_lock
        self.verification = ProbeSurfaceVerificationCoordinator()

    def begin(self, draft, request_id: str) -> None:
        refinement = self.refinement_controller.require_refinement(draft)
        if not refinement.stage_is_approved(
            RefinementStage.SAFE_APPROACH
        ):
            raise ValueError(
                "Approve the safe approach before surface verification"
            )
        if not refinement.stage_is_approved(
            RefinementStage.ALIGNMENT
        ):
            raise ValueError(
                "Approve the aligned pre-approach before "
                "surface verification"
            )
        if (
            refinement.motion_states[RefinementStage.ALIGNMENT]
            != RefinementMotionState.REACHED
        ):
            raise ValueError(
                "Reach the aligned pre-approach before "
                "surface verification"
            )
        refinement.active_stage = RefinementStage.PROBE
        verification = self.verification.begin(
            refinement,
            request_id,
        )
        with self.state_lock:
            draft.surface_verification = verification

    def evaluate(
        self,
        draft,
        request_id: str,
        samples,
        achieved_pose_object,
    ):
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        return self.verification.evaluate_samples(
            verification,
            refinement,
            samples,
            achieved_pose_object=achieved_pose_object,
        )

    def evaluate_estimated_distance(
        self,
        draft,
        request_id: str,
        estimated_distance_m: float,
        achieved_pose_object,
    ):
        """Evaluate frozen-surface progress without another depth sample."""
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        return self.verification.evaluate_estimated_distance(
            verification,
            refinement,
            estimated_distance_m,
            achieved_pose_object,
        )

    def mark_correction_started(
        self,
        draft,
        request_id: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        self.verification.mark_correction_started(
            verification,
            refinement,
        )

    def mark_correction_succeeded(
        self,
        draft,
        request_id: str,
    ) -> None:
        verification = self._session(draft, request_id)
        self.verification.mark_correction_succeeded(
            verification
        )

    def resume_sampling(
        self,
        draft,
        request_id: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        if verification.state is SurfaceVerificationState.RECOVERY_REQUIRED:
            self.verification.restart_after_retraction(
                verification,
                refinement,
            )
            return
        self.verification.resume_sampling(verification)

    def fail_sampling(
        self,
        draft,
        request_id: str,
        detail: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        self.verification.mark_sampling_failed(
            verification,
            refinement,
            detail,
        )

    def fail_correction(
        self,
        draft,
        request_id: str,
        detail: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        self.verification.mark_correction_failed(
            verification,
            refinement,
            detail,
        )

    def abort(
        self,
        draft,
        request_id: str,
        detail: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        self.verification.abort(
            verification,
            refinement,
            detail,
        )

    def cancel(
        self,
        draft,
        request_id: str,
    ) -> None:
        refinement = self.refinement_controller.require_refinement(
            draft
        )
        verification = self._session(draft, request_id)
        self.verification.cancel(
            verification,
            refinement,
        )

    @staticmethod
    def active_request_id(draft) -> str:
        verification = draft.surface_verification
        if verification is None or not verification.active:
            return ""
        return verification.request_id

    @staticmethod
    def _session(draft, request_id: str):
        verification = draft.surface_verification
        if verification is None:
            raise RuntimeError(
                "Surface verification is not active"
            )
        if verification.request_id != request_id:
            raise RuntimeError(
                "Surface verification request ID does not match"
            )
        return verification


__all__ = ["ProbeSurfaceVerificationController"]
