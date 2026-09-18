"""Regression coverage for flexible alignment motion ordering."""

from fault_detector_spot.application.commanding.request_identity import (
    new_request_id,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)


def pose(x=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=orientation or QuaternionData.identity(),
    )


def setup():
    target = ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(0.03),
        aligned_preapproach_pose_object=pose(0.08),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.08,
        direction_source="surface_fit",
    )
    return initialize_reference_probe_setup(target)


def test_orientation_before_candidate_remains_oriented():
    session = ProbeRefinementSession.create(setup())
    session.seed_safe_approach_from_current_pose(pose(0.30))
    request_id = new_request_id()
    achieved = pose(
        0.30,
        QuaternionData(x=0.0, y=0.0, z=1.0, w=0.0),
    )
    session.begin_motion(
        PendingRefinementMotion(
            request_id=request_id,
            stage=RefinementStage.ALIGNMENT,
            purpose="surface orientation",
            target_pose_object=achieved,
            updates_candidate=True,
            verify_achieved_pose=False,
        )
    )
    session.complete_alignment_orientation(request_id, achieved)

    assert session.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.ORIENTED
    )


def test_orientation_after_candidate_keeps_reached_state():
    session = ProbeRefinementSession.create(setup())
    session.seed_safe_approach_from_current_pose(pose(0.30))

    candidate = session.candidate_pose(RefinementStage.ALIGNMENT)
    move_request = new_request_id()
    session.begin_motion(
        PendingRefinementMotion(
            request_id=move_request,
            stage=RefinementStage.ALIGNMENT,
            purpose="alignment",
            target_pose_object=candidate,
            updates_candidate=False,
            verify_achieved_pose=True,
        )
    )
    session.complete_motion(move_request, candidate)

    orientation_request = new_request_id()
    achieved = pose(
        candidate.position.x,
        QuaternionData(x=0.0, y=0.0, z=1.0, w=0.0),
    )
    session.begin_motion(
        PendingRefinementMotion(
            request_id=orientation_request,
            stage=RefinementStage.ALIGNMENT,
            purpose="surface orientation",
            target_pose_object=achieved,
            updates_candidate=True,
            verify_achieved_pose=False,
            preserve_reached_state=True,
        )
    )
    session.complete_alignment_orientation(
        orientation_request,
        achieved,
    )

    assert session.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )
    updated = session.candidate_pose(RefinementStage.ALIGNMENT)
    assert updated.position == candidate.position
    assert updated.orientation == achieved.orientation
