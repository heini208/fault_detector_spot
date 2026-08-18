"""Tests for immutable probe setup state rendering models."""

import pytest

from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.ui.ros.probe_setup_state_adapter import (
    ProbeRefinementSnapshot,
    probe_setup_state_to_view,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)


def pose(message, x):
    message.position.x = x
    message.orientation.w = 1.0


def complete_state():
    state = ProbeSetupState()
    state.has_reference_pixel = True
    state.reference_pixel_u = 20
    state.reference_pixel_v = 30
    state.has_surface_point = True
    state.surface_point_camera.x = 0.4
    state.surface_frame = "hand_depth"
    state.mapped_depth_u = 10
    state.mapped_depth_v = 15
    state.sampled_depth_u = 11
    state.sampled_depth_v = 15
    state.depth_m = 0.4
    state.has_surface_normal = True
    state.surface_normal_camera.x = -1.0
    state.surface_normal_sample_count = 24
    state.surface_normal_rmse_m = 0.001
    state.has_approach_direction = True
    state.approach_direction_camera.x = -1.0
    state.approach_source = "surface_fit"
    state.has_probe_setup = True
    state.surface_point_object.x = 0.4
    state.outward_direction_object.x = 1.0
    state.target_surface_distance_m = 0.10
    state.aligned_preapproach_distance_m = 0.20
    pose(state.calculated_safe_approach_pose_object, 0.60)
    pose(state.calculated_aligned_preapproach_pose_object, 0.60)
    pose(state.calculated_probe_pose_object, 0.50)
    pose(state.safe_approach_pose_object, 0.80)
    pose(state.aligned_preapproach_pose_object, 0.60)
    pose(state.probe_pose_object, 0.50)
    state.safe_approach_approved = True
    return state


def refinement_state():
    state = complete_state()
    state.refinement_active = True
    state.refinement_stage = ProbeSetupState.REFINEMENT_STAGE_ALIGNMENT
    pose(state.safe_approach_candidate_pose_object, 0.81)
    pose(state.aligned_preapproach_candidate_pose_object, 0.61)
    pose(state.probe_candidate_pose_object, 0.51)
    state.safe_approach_motion_state = ProbeSetupState.MOTION_REACHED
    state.alignment_motion_state = ProbeSetupState.MOTION_MOVING
    state.probe_motion_state = ProbeSetupState.MOTION_NOT_TESTED
    state.motion_pending = True
    state.motion_request_id = "1" * 32
    state.refinement_recovery_required = True
    state.refinement_recovery_message = "Retract"
    return state


def test_transport_state_becomes_read_only_geometry_view():
    view = probe_setup_state_to_view(complete_state())

    assert view.projected_point.requested_pixel.u == 20
    assert view.projected_point.mapped_pixel.u == 10
    assert view.surface_normal.sample_count == 24
    assert view.approach_direction.source == "surface_fit"
    assert view.calculated_setup.safe_approach_pose_object.position.x == 0.60
    assert view.setup.safe_approach_pose_object.position.x == 0.80
    assert view.setup.safe_approach_approved


def test_empty_state_has_no_geometry_view():
    view = probe_setup_state_to_view(ProbeSetupState())

    assert view.projected_point is None
    assert view.surface_normal is None
    assert view.approach_direction is None
    assert view.surface_target is None
    assert view.calculated_setup is None
    assert view.setup is None
    assert view.refinement is None


def test_refinement_state_becomes_immutable_server_snapshot():
    refinement = probe_setup_state_to_view(refinement_state()).refinement

    assert isinstance(refinement.snapshot, ProbeRefinementSnapshot)
    assert (
        refinement.snapshot.server_active_stage
        is RefinementStage.ALIGNMENT
    )
    assert refinement.candidate_pose(
        RefinementStage.SAFE_APPROACH
    ).position.x == 0.81
    assert (
        refinement.motion_states[RefinementStage.ALIGNMENT]
        is RefinementMotionState.MOVING
    )
    assert refinement.pending_motion.request_id == "1" * 32
    assert refinement.recovery_required
    assert not refinement.surface_distance_verified


def test_wizard_navigation_cannot_change_server_refinement_snapshot():
    refinement = probe_setup_state_to_view(refinement_state()).refinement

    refinement.active_stage = RefinementStage.SAFE_APPROACH
    with pytest.raises(AttributeError):
        refinement.surface_distance_verified = True

    assert refinement.active_stage is RefinementStage.SAFE_APPROACH
    assert (
        refinement.snapshot.server_active_stage
        is RefinementStage.ALIGNMENT
    )
    assert not refinement.surface_distance_verified

    with pytest.raises(TypeError):
        refinement.motion_states[RefinementStage.ALIGNMENT] = (
            RefinementMotionState.FAILED
        )
