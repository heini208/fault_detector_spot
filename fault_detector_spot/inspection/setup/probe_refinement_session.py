"""State and safety rules for supervised probe-point refinement."""

from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

from fault_detector_spot.application.commanding.request_identity import (
    validate_request_id,
)
from fault_detector_spot.inspection.geometry.pose import (
    add_vectors,
    scale_vector,
)
from fault_detector_spot.inspection.geometry.rotation import rotate_vector
from fault_detector_spot.inspection.model.models import PoseData, Vector3Data
from .reference_probe_setup import (
    ReferenceProbeSetup,
    derive_aligned_preapproach_pose,
)


class RefinementStage(str, Enum):
    """Ordered probe refinement stages."""

    SAFE_APPROACH = "safe_approach"
    ALIGNMENT = "alignment"
    PROBE = "probe"


class RefinementMotionState(str, Enum):
    """Observable state of one required movement."""

    NOT_TESTED = "Not Tested"
    MOVING = "Moving"
    REACHED = "Reached"
    FAILED = "Failed"


@dataclass(frozen=True)
class PendingRefinementMotion:
    """One correlated movement awaiting a terminal result."""

    request_id: str
    stage: RefinementStage
    purpose: str
    target_pose_object: PoseData
    updates_candidate: bool = True
    command_id: str = "move_to_tag"
    verify_achieved_pose: bool = True


@dataclass
class ProbeRefinementSession:
    """Hold draft geometry without changing a persisted probe point."""

    calculated_setup: ReferenceProbeSetup
    approved_setup: ReferenceProbeSetup
    candidate_poses: Dict[RefinementStage, PoseData] = field(
        default_factory=dict
    )
    approved_poses: Dict[RefinementStage, Optional[PoseData]] = field(
        default_factory=dict
    )
    draft_approved: Dict[RefinementStage, bool] = field(
        default_factory=dict
    )
    motion_states: Dict[RefinementStage, RefinementMotionState] = field(
        default_factory=dict
    )
    active_stage: RefinementStage = RefinementStage.SAFE_APPROACH
    pending_motion: Optional[PendingRefinementMotion] = None
    recovery_required: bool = False
    recovery_message: str = ""
    saved: bool = False

    @classmethod
    def create(
        cls,
        calculated_setup: ReferenceProbeSetup,
        approved_setup: Optional[ReferenceProbeSetup] = None,
    ) -> "ProbeRefinementSession":
        """Create a draft from calculated and previously approved poses."""
        approved = approved_setup or calculated_setup
        calculated = deepcopy(calculated_setup)
        approved = deepcopy(approved)
        candidate_poses = {
            RefinementStage.SAFE_APPROACH: deepcopy(
                approved.safe_approach_pose_object
            ),
            RefinementStage.ALIGNMENT: deepcopy(
                approved.aligned_preapproach_pose_object
            ),
            RefinementStage.PROBE: deepcopy(
                approved.probe_pose_object
            ),
        }
        approved_poses = {
            RefinementStage.SAFE_APPROACH: (
                deepcopy(approved.safe_approach_pose_object)
                if approved.safe_approach_approved
                else None
            ),
            RefinementStage.ALIGNMENT: (
                deepcopy(approved.aligned_preapproach_pose_object)
                if approved.surface_alignment_approved
                else None
            ),
            RefinementStage.PROBE: (
                deepcopy(approved.probe_pose_object)
                if approved.probe_pose_approved
                else None
            ),
        }
        active_stage = cls._first_incomplete_stage(approved_poses)
        return cls(
            calculated_setup=calculated,
            approved_setup=approved,
            candidate_poses=candidate_poses,
            approved_poses=approved_poses,
            draft_approved={
                stage: approved_poses[stage] is not None
                for stage in RefinementStage
            },
            motion_states={
                stage: RefinementMotionState.NOT_TESTED
                for stage in RefinementStage
            },
            active_stage=active_stage,
        )

    @staticmethod
    def _first_incomplete_stage(approved_poses):
        for stage in RefinementStage:
            if approved_poses.get(stage) is None:
                return stage
        return RefinementStage.SAFE_APPROACH

    def calculated_pose(self, stage: RefinementStage) -> PoseData:
        """Return the immutable calculated pose for a stage."""
        values = {
            RefinementStage.SAFE_APPROACH: (
                self.calculated_setup.safe_approach_pose_object
            ),
            RefinementStage.ALIGNMENT: (
                self.calculated_setup.aligned_preapproach_pose_object
            ),
            RefinementStage.PROBE: self.calculated_setup.probe_pose_object,
        }
        return deepcopy(values[stage])

    def candidate_pose(self, stage: RefinementStage) -> PoseData:
        """Return the current unpersisted candidate pose."""
        return deepcopy(self.candidate_poses[stage])

    def approved_pose(
        self,
        stage: RefinementStage,
    ) -> Optional[PoseData]:
        """Return the last explicitly approved pose, if one exists."""
        value = self.approved_poses[stage]
        return deepcopy(value) if value is not None else None

    def set_candidate(
        self,
        stage: RefinementStage,
        achieved_pose_object: PoseData,
    ) -> None:
        """Update draft geometry from a verified achieved tip pose."""
        achieved_pose_object.validate()
        self.saved = False
        if stage == RefinementStage.SAFE_APPROACH:
            self.candidate_poses[stage] = deepcopy(achieved_pose_object)
            self.draft_approved[stage] = False
            return
        if stage == RefinementStage.ALIGNMENT:
            self.candidate_poses[stage] = deepcopy(achieved_pose_object)
            self.candidate_poses[RefinementStage.PROBE] = (
                self._derive_probe_pose(achieved_pose_object)
            )
            self.draft_approved[stage] = False
            self.draft_approved[RefinementStage.PROBE] = False
            return
        self.candidate_poses[stage] = deepcopy(achieved_pose_object)
        self.candidate_poses[RefinementStage.ALIGNMENT] = (
            derive_aligned_preapproach_pose(
                achieved_pose_object,
                self.target_surface_distance_m,
                self.aligned_preapproach_distance_m,
            )
        )
        self.draft_approved[stage] = False

    def seed_safe_approach_from_current_pose(
        self,
        achieved_pose_object: PoseData,
    ) -> None:
        """Start manual safe-pose refinement at the achieved live pose."""
        if self.stage_is_approved(RefinementStage.SAFE_APPROACH):
            raise RuntimeError("Safe approach is already approved")
        self.set_candidate(
            RefinementStage.SAFE_APPROACH,
            achieved_pose_object,
        )
        self.motion_states[RefinementStage.SAFE_APPROACH] = (
            RefinementMotionState.REACHED
        )

    def with_updated_surface_geometry(
        self,
        calculated_setup: ReferenceProbeSetup,
        approved_setup: ReferenceProbeSetup,
    ) -> "ProbeRefinementSession":
        """Replace distance geometry without resetting reached setup stages."""
        if self.pending_motion is not None:
            raise RuntimeError(
                "Surface geometry cannot change during robot movement"
            )
        if self.recovery_required:
            raise RuntimeError(
                "Surface geometry cannot change before retraction"
            )
        updated = self.create(calculated_setup, approved_setup)
        for stage in (
            RefinementStage.SAFE_APPROACH,
            RefinementStage.ALIGNMENT,
        ):
            updated.candidate_poses[stage] = self.candidate_pose(stage)
            updated.motion_states[stage] = self.motion_states[stage]
        updated.set_candidate(
            RefinementStage.ALIGNMENT,
            updated.candidate_poses[RefinementStage.ALIGNMENT],
        )
        updated.approved_poses[RefinementStage.SAFE_APPROACH] = (
            deepcopy(
                approved_setup.safe_approach_pose_object
            )
            if approved_setup.safe_approach_approved
            else None
        )
        updated.approved_poses[RefinementStage.ALIGNMENT] = (
            deepcopy(
                approved_setup.aligned_preapproach_pose_object
            )
            if approved_setup.surface_alignment_approved
            else None
        )
        updated.draft_approved[RefinementStage.SAFE_APPROACH] = (
            approved_setup.safe_approach_approved
        )
        updated.draft_approved[RefinementStage.ALIGNMENT] = (
            approved_setup.surface_alignment_approved
        )
        updated.active_stage = self.active_stage
        return updated

    def approve(
        self,
        stage: RefinementStage,
        achieved_pose_object: PoseData,
    ) -> None:
        """Approve an achieved pose without persisting partial state."""
        self.set_candidate(stage, achieved_pose_object)
        self.approved_poses[stage] = deepcopy(
            self.candidate_poses[stage]
        )
        self.draft_approved[stage] = True

    def stage_is_approved(self, stage: RefinementStage) -> bool:
        """Return whether the current candidate was explicitly approved."""
        return bool(self.draft_approved[stage])

    def begin_motion(
        self,
        motion: PendingRefinementMotion,
    ) -> None:
        """Enter a single correlated movement state."""
        if self.pending_motion is not None:
            raise RuntimeError("A refinement movement is already active")
        validate_request_id(motion.request_id)
        motion.target_pose_object.validate()
        if (
            motion.stage == RefinementStage.ALIGNMENT
            and self.motion_states[RefinementStage.SAFE_APPROACH]
            != RefinementMotionState.REACHED
        ):
            raise RuntimeError(
                "Reach the safe approach during this workflow first"
            )
        if (
            motion.stage == RefinementStage.PROBE
            and self.motion_states[RefinementStage.ALIGNMENT]
            != RefinementMotionState.REACHED
        ):
            raise RuntimeError(
                "Reach the aligned pre-approach during this workflow first"
            )
        self.pending_motion = motion
        self.motion_states[motion.stage] = RefinementMotionState.MOVING

    def complete_motion(
        self,
        request_id: str,
        achieved_pose_object: PoseData,
    ) -> None:
        """Commit a successful motion using its achieved tip pose."""
        motion = self._matching_motion(request_id)
        achieved_pose_object.validate()
        if motion.updates_candidate:
            self.set_candidate(motion.stage, achieved_pose_object)
        self.motion_states[motion.stage] = RefinementMotionState.REACHED
        self.pending_motion = None

    def complete_motion_without_pose_capture(self, request_id: str) -> None:
        """Complete a movement without capturing an object-relative pose."""
        motion = self._matching_motion(request_id)
        if motion.updates_candidate or motion.verify_achieved_pose:
            raise RuntimeError(
                "Pose-verifying motion requires an achieved object pose"
            )
        self.motion_states[motion.stage] = RefinementMotionState.REACHED
        self.pending_motion = None

    def fail_motion(self, request_id: str, message: str) -> None:
        """Fail the current movement without changing its candidate."""
        motion = self._matching_motion(request_id)
        self.motion_states[motion.stage] = RefinementMotionState.FAILED
        self.pending_motion = None

    def _matching_motion(
        self,
        request_id: str,
    ) -> PendingRefinementMotion:
        request_id = validate_request_id(request_id)
        if self.pending_motion is None:
            raise RuntimeError("No refinement movement is active")
        if self.pending_motion.request_id != request_id:
            raise RuntimeError("Motion result request ID does not match")
        return self.pending_motion

    def require_recovery(self, message: str) -> None:
        """Block refinement until a successful retraction completes."""
        self.recovery_required = True
        self.recovery_message = message.strip() or "Retraction required"

    def complete_retraction(self) -> None:
        """Return the session to the aligned pre-approach state."""
        self.recovery_required = False
        self.recovery_message = ""
        self.motion_states[RefinementStage.ALIGNMENT] = (
            RefinementMotionState.REACHED
        )
        self.motion_states[RefinementStage.PROBE] = (
            RefinementMotionState.NOT_TESTED
        )

    def discard_unapproved_candidates(self) -> None:
        """Restore approved or calculated poses without changing storage."""
        if self.recovery_required:
            raise RuntimeError("Retract before closing the workflow")
        for stage in RefinementStage:
            approved = self.approved_poses[stage]
            self.candidate_poses[stage] = deepcopy(
                approved
                if approved is not None
                else self.calculated_pose(stage)
            )
            self.draft_approved[stage] = approved is not None
        self.pending_motion = None
        for stage in RefinementStage:
            self.motion_states[stage] = RefinementMotionState.NOT_TESTED

    @property
    def target_surface_distance_m(self) -> float:
        """Return the desired absolute probe-tip surface distance."""
        return self.calculated_setup.surface_target.target_surface_distance_m

    @property
    def aligned_preapproach_distance_m(self) -> float:
        """Return the absolute aligned pre-approach distance."""
        return (
            self.calculated_setup.surface_target
            .aligned_preapproach_distance_m
        )

    def _derive_probe_pose(self, aligned_pose_object: PoseData) -> PoseData:
        distance_delta = (
            self.aligned_preapproach_distance_m
            - self.target_surface_distance_m
        )
        inward = rotate_vector(
            aligned_pose_object.orientation,
            Vector3Data(x=1.0, y=0.0, z=0.0),
        )
        return PoseData(
            position=add_vectors(
                aligned_pose_object.position,
                scale_vector(inward, distance_delta),
            ),
            orientation=deepcopy(aligned_pose_object.orientation),
        )
