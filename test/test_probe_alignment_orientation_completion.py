"""Regression tests for alignment-orientation refinement completion."""

from threading import RLock
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.commanding.request_identity import (
    new_request_id,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.shared.geometry.models import (
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
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)


def pose(x=0.0, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


def calculated_setup():
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


class AlignmentController(ProbeRefinementController):
    def __init__(self, achieved_pose):
        super().__init__(
            setup_coordinator=SimpleNamespace(),
            object_repository=None,
            motion_state_source=None,
            motion_command_factory=None,
            state_lock=RLock(),
            sensor_attachment_controller=SimpleNamespace(),
        )
        self._achieved_pose = achieved_pose

    def _active_attachment(self, draft=None):
        return SimpleNamespace()

    def current_probe_pose(self, draft, attachment=None):
        return self._achieved_pose


def test_successful_surface_orientation_updates_candidate_orientation():
    setup = calculated_setup()
    refinement = ProbeRefinementSession.create(setup, setup)
    refinement.seed_safe_approach_from_current_pose(pose(x=0.30))
    request_id = new_request_id()
    original = refinement.candidate_pose(RefinementStage.ALIGNMENT)
    achieved = PoseData(
        position=Vector3Data(x=0.30, y=0.0, z=0.0),
        orientation=QuaternionData(x=0.0, y=0.0, z=1.0, w=0.0),
    )
    refinement.begin_motion(
        PendingRefinementMotion(
            request_id=request_id,
            stage=RefinementStage.ALIGNMENT,
            purpose="alignment orientation",
            target_pose_object=achieved,
            updates_candidate=True,
            verify_achieved_pose=False,
        )
    )
    draft = SimpleNamespace(refinement=refinement)
    motion = SimpleNamespace(
        kind=ProbeMotionKind.ORIENT_TO_SURFACE,
        position_tolerance_m=0.01,
        orientation_tolerance_rad=0.10,
    )
    controller = AlignmentController(achieved)
    controller._operations._operations[request_id] = SimpleNamespace(
        payload=(motion, RefinementStage.ALIGNMENT)
    )
    status = SimpleNamespace(
        operation=SimpleNamespace(request_id=request_id),
        state=CommandControllerState.SUCCEEDED,
        buffered_command_count=0,
    )

    controller.handle_terminal_status(status, draft)

    assert refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.ORIENTED
    )
    candidate = refinement.candidate_pose(RefinementStage.ALIGNMENT)
    assert candidate.position == original.position
    assert candidate.orientation == achieved.orientation


@pytest.mark.parametrize("previous_state", [
    RefinementMotionState.REACHED,
    RefinementMotionState.ORIENTED,
    RefinementMotionState.NOT_TESTED,
    RefinementMotionState.FAILED,
])
@pytest.mark.parametrize("state,continuation_safe,preserved", [
    (CommandControllerState.FAILED, True, True),
    (CommandControllerState.FAILED, False, False),
    (CommandControllerState.CANCELLED, True, False),
    (CommandControllerState.STOP_CONFIRMED, True, False),
])
def test_failed_tip_orientation_preserves_prior_state_only_with_execution_proof(
    previous_state, state, continuation_safe, preserved,
):
    setup = calculated_setup()
    refinement = ProbeRefinementSession.create(setup, setup)
    refinement.seed_safe_approach_from_current_pose(pose(x=0.30))
    refinement.motion_states[RefinementStage.ALIGNMENT] = previous_state
    refinement.alignment_candidate_reached = previous_state is RefinementMotionState.REACHED
    candidate = refinement.candidate_pose(RefinementStage.ALIGNMENT)
    request_id = new_request_id()
    refinement.begin_motion(PendingRefinementMotion(
        request_id=request_id,
        stage=RefinementStage.ALIGNMENT,
        purpose="sensor tip surface orientation",
        target_pose_object=candidate,
        verify_achieved_pose=False,
        previous_motion_state=previous_state,
    ))
    controller = AlignmentController(candidate)
    motion = SimpleNamespace(kind=ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE)
    controller._operations._operations[request_id] = SimpleNamespace(
        payload=(motion, RefinementStage.ALIGNMENT),
    )
    status = SimpleNamespace(
        operation=SimpleNamespace(request_id=request_id),
        state=state,
        continuation_safe=continuation_safe,
    )

    controller.handle_terminal_status(status, SimpleNamespace(refinement=refinement))

    assert refinement.pending_motion is None
    assert refinement.candidate_pose(RefinementStage.ALIGNMENT) == candidate
    assert refinement.motion_states[RefinementStage.ALIGNMENT] is (
        previous_state if preserved else RefinementMotionState.FAILED
    )
