"""Convert probe setup transport state into immutable view models."""

from dataclasses import dataclass
from typing import Optional

from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.inspection.model.models import ImagePoint, Vector3Data
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_view_approach_direction import (
    ReferenceApproachDirection,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ProjectedReferencePoint,
)
from fault_detector_spot.inspection.setup.reference_view_surface_normal import (
    ReferenceSurfaceNormal,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data


@dataclass(frozen=True)
class ProbeSetupView:
    """Expose transport state in the existing immutable geometry types."""

    message: ProbeSetupState
    projected_point: Optional[ProjectedReferencePoint]
    surface_normal: Optional[ReferenceSurfaceNormal]
    approach_direction: Optional[ReferenceApproachDirection]
    surface_target: Optional[ReferenceSurfaceTarget]
    calculated_setup: Optional[ReferenceProbeSetup]
    setup: Optional[ReferenceProbeSetup]
    refinement: Optional[ProbeRefinementSession]


def probe_setup_state_to_view(state: ProbeSetupState) -> ProbeSetupView:
    """Build one read-only rendering model from a typed state message."""
    if not isinstance(state, ProbeSetupState):
        raise TypeError("Expected a ProbeSetupState message")
    projected = _projected_point(state)
    normal = _surface_normal(state, projected)
    approach = _approach_direction(state, projected, normal)
    target = _surface_target(state)
    calculated, setup = _probe_setups(state, target)
    refinement = _refinement(state, calculated, setup)
    return ProbeSetupView(
        message=state,
        projected_point=projected,
        surface_normal=normal,
        approach_direction=approach,
        surface_target=target,
        calculated_setup=calculated,
        setup=setup,
        refinement=refinement,
    )


def _projected_point(state):
    if not state.has_reference_pixel or not state.has_surface_point:
        return None
    requested = ImagePoint(
        u=int(state.reference_pixel_u),
        v=int(state.reference_pixel_v),
    )
    return ProjectedReferencePoint(
        requested_pixel=requested,
        mapped_pixel=ImagePoint(
            u=int(state.mapped_depth_u),
            v=int(state.mapped_depth_v),
        ),
        sampled_pixel=ImagePoint(
            u=int(state.sampled_depth_u),
            v=int(state.sampled_depth_v),
        ),
        point_camera=_vector(state.surface_point_camera),
        frame_id=state.surface_frame,
        depth_m=float(state.depth_m),
    )


def _surface_normal(state, projected):
    if projected is None or not state.has_surface_normal:
        return None
    return ReferenceSurfaceNormal(
        projected_point=projected,
        normal_camera=_vector(state.surface_normal_camera),
        sample_count=int(state.surface_normal_sample_count),
        plane_rmse_m=float(state.surface_normal_rmse_m),
    )


def _approach_direction(state, projected, normal):
    if projected is None or not state.has_approach_direction:
        return None
    return ReferenceApproachDirection(
        projected_point=projected,
        direction_camera=_vector(state.approach_direction_camera),
        source=state.approach_source,
        surface_normal=normal,
    )


def _surface_target(state):
    if not state.has_probe_setup:
        return None
    return ReferenceSurfaceTarget(
        surface_point_object=_vector(state.surface_point_object),
        outward_direction_object=_vector(
            state.outward_direction_object
        ),
        target_pose_object=pose_to_pose_data(
            state.calculated_probe_pose_object
        ),
        aligned_preapproach_pose_object=pose_to_pose_data(
            state.calculated_aligned_preapproach_pose_object
        ),
        target_surface_distance_m=float(
            state.target_surface_distance_m
        ),
        aligned_preapproach_distance_m=float(
            state.aligned_preapproach_distance_m
        ),
        direction_source=state.approach_source,
    )


def _probe_setups(state, target):
    if target is None:
        return None, None
    calculated = ReferenceProbeSetup(
        surface_target=target,
        safe_approach_pose_object=pose_to_pose_data(
            state.calculated_safe_approach_pose_object
        ),
        aligned_preapproach_pose_object=pose_to_pose_data(
            state.calculated_aligned_preapproach_pose_object
        ),
        probe_pose_object=pose_to_pose_data(
            state.calculated_probe_pose_object
        ),
    )
    setup = ReferenceProbeSetup(
        surface_target=target,
        safe_approach_pose_object=pose_to_pose_data(
            state.safe_approach_pose_object
        ),
        aligned_preapproach_pose_object=pose_to_pose_data(
            state.aligned_preapproach_pose_object
        ),
        probe_pose_object=pose_to_pose_data(state.probe_pose_object),
        safe_approach_approved=state.safe_approach_approved,
        surface_alignment_approved=state.surface_alignment_approved,
        probe_pose_approved=state.probe_pose_approved,
    )
    return calculated, setup


def _vector(message):
    return Vector3Data(
        x=float(message.x),
        y=float(message.y),
        z=float(message.z),
    )


def _refinement(state, calculated, setup):
    if not state.refinement_active or calculated is None or setup is None:
        return None
    refinement = ProbeRefinementSession.create(calculated, setup)
    refinement.candidate_poses = {
        RefinementStage.SAFE_APPROACH: pose_to_pose_data(
            state.safe_approach_candidate_pose_object
        ),
        RefinementStage.ALIGNMENT: pose_to_pose_data(
            state.aligned_preapproach_candidate_pose_object
        ),
        RefinementStage.PROBE: pose_to_pose_data(
            state.probe_candidate_pose_object
        ),
    }
    stages = {
        ProbeSetupState.REFINEMENT_STAGE_SAFE_APPROACH: (
            RefinementStage.SAFE_APPROACH
        ),
        ProbeSetupState.REFINEMENT_STAGE_ALIGNMENT: RefinementStage.ALIGNMENT,
        ProbeSetupState.REFINEMENT_STAGE_PROBE: RefinementStage.PROBE,
    }
    refinement.active_stage = stages.get(
        int(state.refinement_stage),
        RefinementStage.SAFE_APPROACH,
    )
    motion_states = {
        ProbeSetupState.MOTION_NOT_TESTED: RefinementMotionState.NOT_TESTED,
        ProbeSetupState.MOTION_MOVING: RefinementMotionState.MOVING,
        ProbeSetupState.MOTION_REACHED: RefinementMotionState.REACHED,
        ProbeSetupState.MOTION_FAILED: RefinementMotionState.FAILED,
    }
    refinement.motion_states = {
        RefinementStage.SAFE_APPROACH: motion_states[
            int(state.safe_approach_motion_state)
        ],
        RefinementStage.ALIGNMENT: motion_states[
            int(state.alignment_motion_state)
        ],
        RefinementStage.PROBE: motion_states[
            int(state.probe_motion_state)
        ],
    }
    refinement.recovery_required = bool(
        state.refinement_recovery_required
    )
    refinement.recovery_message = state.refinement_recovery_message
    refinement.surface_distance_verified = (
        state.surface_verification_state
        == ProbeSetupState.SURFACE_VERIFICATION_CONVERGED
    )
    if state.motion_pending:
        stage = refinement.active_stage
        refinement.pending_motion = PendingRefinementMotion(
            request_id=state.motion_request_id,
            stage=stage,
            purpose="probe setup motion",
            target_pose_object=refinement.candidate_pose(stage),
            updates_candidate=True,
        )
    return refinement


__all__ = ["ProbeSetupView", "probe_setup_state_to_view"]
