"""Tests for the ordered supervised probe-refinement state machine."""

from copy import deepcopy
import math

import pytest

from fault_detector_spot.request_identity import new_request_id
from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.probe_refinement_session import (
    PendingRefinementMotion,
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.reference_probe_setup import (
    approve_probe_pose,
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    """Create one valid object-frame sensor-tip pose."""
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def yaw_quaternion(degrees):
    """Create a normalized yaw-only quaternion."""
    half_angle = math.radians(degrees) * 0.5
    return QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(half_angle),
        w=math.cos(half_angle),
    )


def calculated_setup():
    """Create geometry with five centimetres of axial separation."""
    target = ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(x=0.03),
        aligned_preapproach_pose_object=pose(x=0.08),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.08,
        direction_source="surface_fit",
    )
    return initialize_reference_probe_setup(target)


def motion(stage, target, purpose="test", correction=0.0):
    """Create a pending motion for state-machine tests."""
    return PendingRefinementMotion(
        request_id=new_request_id(),
        stage=stage,
        purpose=purpose,
        target_pose_object=target,
        axial_correction_m=correction,
    )


def reach(session, stage, target):
    """Begin and complete one verified motion."""
    pending = motion(stage, target)
    session.begin_motion(pending)
    session.complete_motion(pending.request_id, target)


def test_session_starts_at_first_unapproved_stage():
    setup = calculated_setup()
    approved = approve_safe_approach_pose(setup, pose(x=0.30))

    session = ProbeRefinementSession.create(setup, approved)

    assert session.active_stage == RefinementStage.ALIGNMENT
    assert session.stage_is_approved(RefinementStage.SAFE_APPROACH)
    assert not session.stage_is_approved(RefinementStage.ALIGNMENT)


def test_manual_safe_approach_starts_at_live_pose_without_approval():
    session = ProbeRefinementSession.create(calculated_setup())
    current = pose(x=0.42, y=0.10, z=0.30)

    session.seed_safe_approach_from_current_pose(current)

    assert session.candidate_pose(RefinementStage.SAFE_APPROACH) == current
    assert session.motion_states[RefinementStage.SAFE_APPROACH] == (
        RefinementMotionState.REACHED
    )
    assert not session.stage_is_approved(RefinementStage.SAFE_APPROACH)


def test_motion_requires_safe_then_aligned_in_current_workflow():
    session = ProbeRefinementSession.create(calculated_setup())

    with pytest.raises(RuntimeError, match="safe approach"):
        session.begin_motion(
            motion(RefinementStage.ALIGNMENT, pose(x=0.08))
        )

    reach(session, RefinementStage.SAFE_APPROACH, pose(x=0.30))
    assert session.motion_states[RefinementStage.SAFE_APPROACH] == (
        RefinementMotionState.REACHED
    )

    with pytest.raises(RuntimeError, match="aligned pre-approach"):
        session.begin_motion(
            motion(RefinementStage.PROBE, pose(x=0.06))
        )

    reach(session, RefinementStage.ALIGNMENT, pose(x=0.08))
    session.begin_motion(motion(RefinementStage.PROBE, pose(x=0.06)))
    assert session.motion_states[RefinementStage.PROBE] == (
        RefinementMotionState.MOVING
    )


def test_alignment_candidate_derives_probe_with_shared_orientation():
    session = ProbeRefinementSession.create(calculated_setup())
    aligned = pose(
        x=0.20,
        y=0.10,
        z=0.30,
        orientation=yaw_quaternion(90.0),
    )

    session.set_candidate(RefinementStage.ALIGNMENT, aligned)
    probe = session.candidate_pose(RefinementStage.PROBE)

    assert probe.position.x == pytest.approx(0.20)
    assert probe.position.y == pytest.approx(0.05)
    assert probe.position.z == pytest.approx(0.30)
    assert probe.orientation == aligned.orientation


def test_axial_correction_is_bounded_and_keeps_aligned_candidate():
    session = ProbeRefinementSession.create(calculated_setup())
    reach(session, RefinementStage.SAFE_APPROACH, pose(x=0.30))
    reach(session, RefinementStage.ALIGNMENT, pose(x=0.08))
    aligned_before = session.candidate_pose(RefinementStage.ALIGNMENT)

    pending = motion(
        RefinementStage.PROBE,
        pose(x=0.06),
        correction=0.02,
    )
    session.begin_motion(pending)
    session.complete_motion(pending.request_id, pose(x=0.06))

    assert session.recovery_required
    assert session.cumulative_inward_travel_m == pytest.approx(0.02)
    assert session.candidate_pose(RefinementStage.ALIGNMENT) == aligned_before
    assert session.candidate_pose(RefinementStage.PROBE) == pose(x=0.06)

    with pytest.raises(ValueError, match="configured limit"):
        session.begin_motion(
            motion(
                RefinementStage.PROBE,
                pose(x=-0.01),
                correction=0.05,
            )
        )


def test_verified_probe_rebuilds_retraction_pose_and_requires_approval():
    session = ProbeRefinementSession.create(calculated_setup())
    reach(session, RefinementStage.SAFE_APPROACH, pose(x=0.30))
    reach(session, RefinementStage.ALIGNMENT, pose(x=0.08))
    achieved = pose(
        x=0.04,
        y=0.02,
        orientation=yaw_quaternion(10.0),
    )

    session.mark_surface_verified(achieved)

    aligned = session.candidate_pose(RefinementStage.ALIGNMENT)
    assert session.surface_distance_verified
    assert not session.stage_is_approved(RefinementStage.PROBE)
    assert aligned.orientation == achieved.orientation
    assert aligned.position.x == pytest.approx(
        achieved.position.x + 0.05 * math.cos(math.radians(10.0))
    )
    assert aligned.position.y == pytest.approx(
        achieved.position.y + 0.05 * math.sin(math.radians(10.0))
    )


def test_discard_restores_only_explicitly_approved_candidates():
    setup = calculated_setup()
    approved = approve_safe_approach_pose(setup, pose(x=0.30))
    approved = approve_surface_alignment_pose(approved, pose(x=0.08))
    approved = approve_probe_pose(approved, pose(x=0.03))
    session = ProbeRefinementSession.create(setup, approved)
    approved_alignment = deepcopy(
        session.approved_pose(RefinementStage.ALIGNMENT)
    )
    session.set_candidate(RefinementStage.ALIGNMENT, pose(x=0.10))

    session.discard_unapproved_candidates()

    assert session.candidate_pose(
        RefinementStage.ALIGNMENT
    ) == approved_alignment
    assert session.stage_is_approved(RefinementStage.ALIGNMENT)
    assert all(
        state == RefinementMotionState.NOT_TESTED
        for state in session.motion_states.values()
    )


def test_successful_retraction_clears_only_recovery_evidence():
    session = ProbeRefinementSession.create(calculated_setup())
    session.require_recovery("inward motion")
    session.cumulative_inward_travel_m = 0.04

    session.complete_retraction()

    assert not session.recovery_required
    assert session.cumulative_inward_travel_m == 0.0
    assert session.motion_states[RefinementStage.ALIGNMENT] == (
        RefinementMotionState.REACHED
    )
    assert session.motion_states[RefinementStage.PROBE] == (
        RefinementMotionState.NOT_TESTED
    )


def test_stale_motion_result_cannot_complete_current_request():
    session = ProbeRefinementSession.create(calculated_setup())
    pending = motion(
        RefinementStage.SAFE_APPROACH,
        pose(x=0.30),
    )
    session.begin_motion(pending)

    with pytest.raises(RuntimeError, match="request ID does not match"):
        session.complete_motion(new_request_id(), pose(x=0.30))

    assert session.pending_motion is pending
    assert session.motion_states[RefinementStage.SAFE_APPROACH] == (
        RefinementMotionState.MOVING
    )


def test_relative_motion_completes_without_changing_candidate_pose():
    session = ProbeRefinementSession.create(calculated_setup())
    candidate = session.candidate_pose(RefinementStage.SAFE_APPROACH)
    pending = PendingRefinementMotion(
        request_id=new_request_id(),
        stage=RefinementStage.SAFE_APPROACH,
        purpose="fine adjustment",
        target_pose_object=candidate,
        updates_candidate=False,
        command_id="move_arm_relative",
        verify_achieved_pose=False,
    )
    session.begin_motion(pending)

    session.complete_motion_without_pose_capture(pending.request_id)

    assert session.pending_motion is None
    assert session.candidate_pose(RefinementStage.SAFE_APPROACH) == candidate
    assert session.motion_states[RefinementStage.SAFE_APPROACH] == (
        RefinementMotionState.REACHED
    )
