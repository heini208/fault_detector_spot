"""Regression tests for manual initial safe-approach refinement."""

from threading import RLock
from types import SimpleNamespace

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_safe_approach_pose,
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


class IdleCommandController:
    active_request_id = ""
    queued_request_ids = ()


class SetupCoordinatorStub:
    command_controller = IdleCommandController()


class ActiveAttachmentController:
    def require_motion_attachment(self):
        return SimpleNamespace(
            sensor_id="active_probe",
            motion_sensor_id="active_probe",
            hand_to_probe=lambda: PoseData.identity(),
        )


class CurrentPoseRefinementController(ProbeRefinementController):
    def __init__(self, current_pose):
        super().__init__(
            setup_coordinator=SetupCoordinatorStub(),
            object_repository=None,
            motion_state_source=None,
            motion_command_factory=None,
            state_lock=RLock(),
            sensor_attachment_controller=ActiveAttachmentController(),
        )
        self._current_pose = current_pose

    def current_probe_pose(self, draft, attachment=None):
        return self._current_pose


def draft_for(setup):
    return SimpleNamespace(
        geometry=SimpleNamespace(probe_setup=calculated_setup()),
        setup=setup,
        refinement=None,
        surface_verification=object(),
    )


def test_new_safe_approach_starts_from_current_live_probe_pose():
    setup = calculated_setup()
    current = pose(x=0.42, y=0.10, z=0.30)
    draft = draft_for(setup)
    controller = CurrentPoseRefinementController(current)

    controller.begin(draft)

    refinement = draft.refinement
    assert refinement.active_stage is RefinementStage.SAFE_APPROACH
    assert refinement.candidate_pose(RefinementStage.SAFE_APPROACH) == current
    assert refinement.motion_states[RefinementStage.SAFE_APPROACH] is (
        RefinementMotionState.REACHED
    )
    assert not refinement.stage_is_approved(RefinementStage.SAFE_APPROACH)
    assert draft.surface_verification is None


def test_existing_approved_safe_pose_is_not_replaced_by_current_pose():
    calculated = calculated_setup()
    approved_pose = pose(x=0.30)
    setup = approve_safe_approach_pose(calculated, approved_pose)
    current = pose(x=0.90)
    draft = SimpleNamespace(
        geometry=SimpleNamespace(probe_setup=calculated),
        setup=setup,
        refinement=None,
        surface_verification=None,
    )
    controller = CurrentPoseRefinementController(current)

    controller.begin(draft)

    refinement = draft.refinement
    assert (
        refinement.candidate_pose(RefinementStage.SAFE_APPROACH)
        == approved_pose
    )
    assert refinement.motion_states[RefinementStage.SAFE_APPROACH] is (
        RefinementMotionState.NOT_TESTED
    )
    assert refinement.stage_is_approved(RefinementStage.SAFE_APPROACH)
