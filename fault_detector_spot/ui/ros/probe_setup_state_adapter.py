"""Convert probe setup transport state into immutable view models."""

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Optional

from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.inspection.model.models import ImagePoint, Vector3Data
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
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
class ProbePendingMotionView:
    """Expose pending motion correlation without domain workflow ownership."""

    request_id: str
    stage: RefinementStage
    target_pose_object: object


@dataclass(frozen=True)
class ProbeRefinementSnapshot:
    """Capture authoritative refinement state received from the server."""

    server_active_stage: RefinementStage
    calculated_poses: Mapping
    candidate_poses: Mapping
    approved_poses: Mapping
    motion_states: Mapping
    motion_pending: bool
    motion_request_id: str
    recovery_required: bool
    recovery_message: str
    target_surface_distance_m: float
    aligned_preapproach_distance_m: float


class ProbeRefinementPresentation:
    """Bind an immutable server snapshot to local wizard navigation."""

    def __init__(self, snapshot: ProbeRefinementSnapshot):
        if not isinstance(snapshot, ProbeRefinementSnapshot):
            raise TypeError("Expected a ProbeRefinementSnapshot")
        self._snapshot = snapshot
        self._active_stage = snapshot.server_active_stage
        self._pending_motion = (
            ProbePendingMotionView(
                request_id=snapshot.motion_request_id,
                stage=snapshot.server_active_stage,
                target_pose_object=snapshot.candidate_poses[
                    snapshot.server_active_stage
                ],
            )
            if snapshot.motion_pending
            else None
        )

    @property
    def snapshot(self) -> ProbeRefinementSnapshot:
        return self._snapshot

    @property
    def active_stage(self) -> RefinementStage:
        return self._active_stage

    @active_stage.setter
    def active_stage(self, stage: RefinementStage) -> None:
        if not isinstance(stage, RefinementStage):
            raise TypeError("Expected a RefinementStage")
        self._active_stage = stage

    @property
    def pending_motion(self) -> Optional[ProbePendingMotionView]:
        return self._pending_motion

    @property
    def motion_states(self) -> Mapping:
        return self._snapshot.motion_states

    @property
    def recovery_required(self) -> bool:
        return self._snapshot.recovery_required

    @property
    def recovery_message(self) -> str:
        return self._snapshot.recovery_message

    @property
    def target_surface_distance_m(self) -> float:
        return self._snapshot.target_surface_distance_m

    @property
    def aligned_preapproach_distance_m(self) -> float:
        return self._snapshot.aligned_preapproach_distance_m

    @property
    def candidate_poses(self) -> Mapping:
        return self._snapshot.candidate_poses

    def calculated_pose(self, stage: RefinementStage):
        return self._snapshot.calculated_poses[stage]

    def candidate_pose(self, stage: RefinementStage):
        return self._snapshot.candidate_poses[stage]

    def approved_pose(self, stage: RefinementStage):
        return self._snapshot.approved_poses[stage]

    def stage_is_approved(self, stage: RefinementStage) -> bool:
        return self.approved_pose(stage) is not None


@dataclass(frozen=True)
class ProbeSetupView:
    """Expose transport state in immutable rendering models."""

    message: ProbeSetupState
    projected_point: Optional[ProjectedReferencePoint]
    surface_normal: Optional[ReferenceSurfaceNormal]
    approach_direction: Optional[ReferenceApproachDirection]
    surface_target: Optional[ReferenceSurfaceTarget]
    calculated_setup: Optional[ReferenceProbeSetup]
    setup: Optional[ReferenceProbeSetup]
    refinement: Optional[ProbeRefinementPresentation]


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
    stage = _refinement_stage(state.refinement_stage)
    calculated_poses = MappingProxyType(
        {
            RefinementStage.SAFE_APPROACH: (
                calculated.safe_approach_pose_object
            ),
            RefinementStage.ALIGNMENT: (
                calculated.aligned_preapproach_pose_object
            ),
            RefinementStage.PROBE: calculated.probe_pose_object,
        }
    )
    candidate_poses = MappingProxyType(
        {
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
    )
    approved_poses = MappingProxyType(
        {
            RefinementStage.SAFE_APPROACH: (
                setup.safe_approach_pose_object
                if setup.safe_approach_approved
                else None
            ),
            RefinementStage.ALIGNMENT: (
                setup.aligned_preapproach_pose_object
                if setup.surface_alignment_approved
                else None
            ),
            RefinementStage.PROBE: (
                setup.probe_pose_object
                if setup.probe_pose_approved
                else None
            ),
        }
    )
    motion_states = MappingProxyType(
        {
            RefinementStage.SAFE_APPROACH: _motion_state(
                state.safe_approach_motion_state
            ),
            RefinementStage.ALIGNMENT: _motion_state(
                state.alignment_motion_state
            ),
            RefinementStage.PROBE: _motion_state(
                state.probe_motion_state
            ),
        }
    )
    snapshot = ProbeRefinementSnapshot(
        server_active_stage=stage,
        calculated_poses=calculated_poses,
        candidate_poses=candidate_poses,
        approved_poses=approved_poses,
        motion_states=motion_states,
        motion_pending=bool(state.motion_pending),
        motion_request_id=state.motion_request_id,
        recovery_required=bool(state.refinement_recovery_required),
        recovery_message=state.refinement_recovery_message,
        target_surface_distance_m=float(
            state.target_surface_distance_m
        ),
        aligned_preapproach_distance_m=float(
            state.aligned_preapproach_distance_m
        ),
    )
    return ProbeRefinementPresentation(snapshot)


def _refinement_stage(value):
    stages = {
        ProbeSetupState.REFINEMENT_STAGE_SAFE_APPROACH: (
            RefinementStage.SAFE_APPROACH
        ),
        ProbeSetupState.REFINEMENT_STAGE_ALIGNMENT: RefinementStage.ALIGNMENT,
        ProbeSetupState.REFINEMENT_STAGE_PROBE: RefinementStage.PROBE,
    }
    return stages.get(int(value), RefinementStage.SAFE_APPROACH)


def _motion_state(value):
    states = {
        ProbeSetupState.MOTION_NOT_TESTED: RefinementMotionState.NOT_TESTED,
        ProbeSetupState.MOTION_MOVING: RefinementMotionState.MOVING,
        ProbeSetupState.MOTION_REACHED: RefinementMotionState.REACHED,
        ProbeSetupState.MOTION_FAILED: RefinementMotionState.FAILED,
    }
    return states[int(value)]


__all__ = [
    "ProbePendingMotionView",
    "ProbeRefinementPresentation",
    "ProbeRefinementSnapshot",
    "ProbeSetupView",
    "probe_setup_state_to_view",
]
