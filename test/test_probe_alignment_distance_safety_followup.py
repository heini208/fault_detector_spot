"""Regression tests for aligned-distance and camera-clearance safety."""

from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.coordinators.probe_refinement_controller import (
    ProbeRefinementController,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_geometry_editor import (
    ProbeGeometryEditor,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeGeometryResult,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    approve_safe_approach_pose,
    approve_surface_alignment_pose,
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


def make_setup(target_distance=0.03, aligned_distance=0.10):
    target = ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(x=target_distance),
        aligned_preapproach_pose_object=pose(x=aligned_distance),
        target_surface_distance_m=target_distance,
        aligned_preapproach_distance_m=aligned_distance,
        direction_source="surface_fit",
    )
    return initialize_reference_probe_setup(target)


def approved_setup(target_distance=0.03, aligned_distance=0.10):
    value = make_setup(target_distance, aligned_distance)
    value = approve_safe_approach_pose(value, pose(x=0.30))
    return approve_surface_alignment_pose(
        value,
        pose(x=aligned_distance, y=0.02),
    )


def test_previous_alignment_approval_is_retained_as_history():
    previous = approved_setup(aligned_distance=0.10)
    updated = make_setup(aligned_distance=0.15)

    retained = ProbeGeometryEditor._retained_distance_approvals(
        updated,
        previous,
    )

    assert retained.safe_approach_approved
    assert retained.surface_alignment_approved
    assert not retained.probe_pose_approved
    assert retained.aligned_preapproach_pose_object == (
        previous.aligned_preapproach_pose_object
    )


def test_target_only_change_retains_alignment_approval():
    previous = approved_setup(target_distance=0.03, aligned_distance=0.10)
    updated = make_setup(target_distance=0.02, aligned_distance=0.10)

    retained = ProbeGeometryEditor._retained_distance_approvals(
        updated,
        previous,
    )

    assert retained.safe_approach_approved
    assert retained.surface_alignment_approved
    assert not retained.probe_pose_approved


def test_active_refinement_shifts_candidate_when_aligned_distance_changes():
    previous = approved_setup(aligned_distance=0.10)
    refinement = ProbeRefinementSession.create(previous, previous)
    refinement.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    refinement.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    previous_candidate = refinement.candidate_pose(
        RefinementStage.ALIGNMENT
    )
    updated_calculated = make_setup(aligned_distance=0.15)
    geometry = SimpleNamespace(probe_setup=updated_calculated)
    draft = SimpleNamespace(
        reference_pixel=ImagePoint(u=10, v=10),
        selected_reference_view_id="view",
        selected_object_id="object",
        selected_routine_id="routine",
        setup=previous,
        geometry=None,
        refinement=refinement,
        dirty=False,
        validation_error="",
    )
    editor = ProbeGeometryEditor(None, None)
    editor._resolve_geometry = lambda *args, **kwargs: geometry

    editor.update_geometry(
        draft,
        "surface_fit",
        0.03,
        0.15,
    )

    candidate = draft.refinement.candidate_pose(
        RefinementStage.ALIGNMENT
    )
    assert candidate.position.x == pytest.approx(
        previous_candidate.position.x - 0.05
    )
    assert candidate.position.y == pytest.approx(
        previous_candidate.position.y
    )
    assert candidate.orientation == previous_candidate.orientation
    assert draft.refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.NOT_TESTED
    )
    assert draft.refinement.motion_states[RefinementStage.PROBE] is (
        RefinementMotionState.NOT_TESTED
    )
    assert not draft.refinement.stage_is_approved(
        RefinementStage.ALIGNMENT
    )


class _Definition:
    def get_routine(self, routine_id):
        assert routine_id == "routine"
        return SimpleNamespace(sensor_id="sensor")


class _ObjectRepository:
    def load(self, object_id):
        assert object_id == "object"
        return _Definition()


class _ClearanceSource:
    def __init__(self, minimum=0.125, live=0.240):
        self.minimum = minimum
        self.live = live
        self.live_calls = 0

    def minimum_aligned_probe_distance_m(self, sensor_id):
        assert sensor_id == "sensor"
        return self.minimum

    def require_hand_camera_clearance(self):
        self.live_calls += 1
        return self.live


class _AttachmentController:
    def require_motion_attachment(self):
        return SimpleNamespace(
            sensor_id="sensor",
            motion_sensor_id="sensor",
            hand_to_probe=lambda: PoseData.identity(),
        )


def test_controller_clamps_configured_alignment_to_camera_safe_minimum():
    original = approved_setup(aligned_distance=0.10)
    geometry = ProbeGeometryResult(
        capture=None,
        projected_point=None,
        surface_normal=None,
        surface_normal_error="",
        approach_direction=None,
        surface_target=original.surface_target,
        probe_setup=original,
    )
    draft = SimpleNamespace(
        selected_object_id="object",
        selected_routine_id="routine",
        geometry=geometry,
        setup=original,
        refinement=None,
        dirty=False,
        validation_error="old",
    )
    controller = ProbeRefinementController.__new__(
        ProbeRefinementController
    )
    controller.object_repository = _ObjectRepository()
    controller.motion_state_source = _ClearanceSource(minimum=0.125)
    controller.sensor_attachment_controller = _AttachmentController()

    minimum = controller._ensure_minimum_camera_clearance_geometry(draft)

    assert minimum == pytest.approx(0.125)
    assert draft.setup.surface_target.aligned_preapproach_distance_m == (
        pytest.approx(0.125)
    )
    assert not draft.setup.surface_alignment_approved
    assert not draft.setup.probe_pose_approved
    assert draft.dirty
    assert draft.validation_error == ""


def test_live_alignment_clearance_gate_delegates_to_state_source():
    controller = ProbeRefinementController.__new__(
        ProbeRefinementController
    )
    source = _ClearanceSource(live=0.235)
    controller.motion_state_source = source

    assert controller._require_live_alignment_camera_clearance() == (
        pytest.approx(0.235)
    )
    assert source.live_calls == 1


def test_explicit_alignment_approval_promotes_failed_pose_to_reached():
    calculated = make_setup(aligned_distance=0.12)
    safe_approved = approve_safe_approach_pose(
        calculated,
        pose(x=0.30),
    )
    refinement = ProbeRefinementSession.create(
        calculated,
        safe_approved,
    )
    refinement.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.FAILED
    )
    draft = SimpleNamespace(
        setup=safe_approved,
        refinement=refinement,
        dirty=False,
        validation_error="",
    )
    controller = ProbeRefinementController.__new__(
        ProbeRefinementController
    )
    controller.state_lock = nullcontext()
    controller.sensor_attachment_controller = _AttachmentController()
    controller.require_physical_lane_idle = lambda: None
    controller._ensure_minimum_camera_clearance_geometry = (
        lambda _draft, _attachment=None: None
    )
    controller._require_live_alignment_camera_clearance = (
        lambda: 0.270
    )
    controller.current_probe_pose = (
        lambda _draft, _attachment=None: pose(x=0.12)
    )

    controller.approve(draft, RefinementStage.ALIGNMENT)

    assert draft.refinement.motion_states[RefinementStage.ALIGNMENT] is (
        RefinementMotionState.REACHED
    )
    assert draft.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    assert draft.setup.surface_alignment_approved
