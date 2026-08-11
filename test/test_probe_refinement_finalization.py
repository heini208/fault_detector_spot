"""Tests for probe refinement finalization state rules."""

from copy import deepcopy

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    ReferenceProbeSetup,
    SurfaceTarget,
)


def _pose(x):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )


def _setup():
    target = SurfaceTarget(
        surface_point_object=Vector3Data(x=0.0, y=0.0, z=0.0),
        outward_direction_object=Vector3Data(x=-1.0, y=0.0, z=0.0),
        target_surface_distance_m=0.02,
        aligned_preapproach_distance_m=0.08,
    )
    return ReferenceProbeSetup(
        surface_target=target,
        safe_approach_pose_object=_pose(-0.15),
        aligned_preapproach_pose_object=_pose(-0.08),
        probe_pose_object=_pose(-0.02),
    )


def test_verified_probe_approval_preserves_distance_verification():
    session = ProbeRefinementSession.create(_setup())
    achieved = _pose(-0.021)
    session.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    session.mark_surface_verified(achieved)

    session.approve_verified_probe()

    assert session.surface_distance_verified
    assert session.stage_is_approved(RefinementStage.PROBE)
    assert session.approved_pose(RefinementStage.PROBE) == achieved


def test_recovery_stays_set_until_complete_retraction():
    session = ProbeRefinementSession.create(_setup())
    session.require_recovery("Probe moved inward")

    assert session.recovery_required
    assert session.recovery_message == "Probe moved inward"

    session.complete_retraction()

    assert not session.recovery_required
    assert session.recovery_message == ""
    assert (
        session.motion_states[RefinementStage.PROBE]
        is RefinementMotionState.NOT_TESTED
    )
