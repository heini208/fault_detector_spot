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
        direction_source="test",
    )
    return initialize_reference_probe_setup(target)


def test_orientation_after_candidate_remains_reached():
    session = ProbeRefinementSession.create(setup())
    session.seed_safe_approach_from_current_pose(pose(0.30))

    move_id = new_request_id()
    candidate = session.candidate_pose(RefinementStage.ALIGNMENT)
    session.begin_motion(
        PendingRefinementMotion(
            request_id=move_id,
            stage=RefinementStage.ALIGNMENT,
            purpose="alignment",
            target_pose_object=candidate,
            updates_candidate=True,
            verify_achieved_pose=True,
        )
    )
    session.complete_motion(move_id, candidate)

    assert session.alignment_candidate_reached
    assert session.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )

    orient_id = new_request_id()
    oriented = PoseData(
        position=candidate.position,
        orientation=QuaternionData(x=0.0, y=0.0, z=1.0, w=0.0),
    )
    session.begin_motion(
        PendingRefinementMotion(
            request_id=orient_id,
            stage=RefinementStage.ALIGNMENT,
            purpose="surface orientation",
            target_pose_object=oriented,
            updates_candidate=True,
            verify_achieved_pose=False,
            preserve_reached_state=False,
        )
    )
    session.complete_alignment_orientation(orient_id, oriented)

    assert session.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )
    assert session.alignment_candidate_reached
