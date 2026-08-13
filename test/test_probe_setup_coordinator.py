"""Tests for server-owned probe authoring and persistence."""

from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
    CommandControllerStatus,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.application.setup.setup_context import (
    StaleSetupContext,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.model.sensor_models import SensorDefinition
from fault_detector_spot.inspection.repository import (
    multi_reference_view_repository,
)
from fault_detector_spot.inspection.repository.sensor_repository import (
    SensorRepository,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind,
    ProbeMotionRequest,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_probe_setup import (
    initialize_reference_probe_setup,
    multiply_quaternions,
    rotate_vector,
)
from fault_detector_spot.inspection.setup import (
    reference_view_surface_target,
)
from fault_detector_spot.inspection.setup.reference_view_surface_target import (
    quaternion_to_rpy,
)


class FakeCommandController:
    def __init__(self):
        self.listeners = []
        self.submitted = []

    @property
    def active_request_id(self):
        return ""

    @property
    def queued_request_ids(self):
        return ()

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def succeed(self, request):
        status = CommandControllerStatus(
            request=request,
            state=CommandControllerState.SUCCEEDED,
        )
        for listener in tuple(self.listeners):
            listener(status)

    def cancel(self, request_id):
        request = next(
            request
            for request in self.submitted
            if request.request_id == request_id
        )
        status = CommandControllerStatus(
            request=request,
            state=CommandControllerState.CANCELLED,
            detail="Cancelled",
        )
        for listener in tuple(self.listeners):
            listener(status)
        return request_id


class FakeMotionStateSource:
    def __init__(self):
        self.pose = pose()
        self.gravity_object_pose = pose()

    def current_probe_pose_object(self, _tag_id, _sensor_id):
        return self.pose

    def gravity_aligned_object_pose(self, _tag_id):
        return self.gravity_object_pose

    def reference_tag(self, _tag_id):
        return None


class FakeMotionCommandFactory:
    def absolute(self, _target, _mounting, _tag):
        return SemanticCommand(
            command_id=CommandID.MOVE_ARM_TO_TAG,
        )

    def relative(self, _frame, _translation, _pitch, _yaw):
        return SemanticCommand(
            command_id=CommandID.MOVE_ARM_RELATIVE,
        )

    @staticmethod
    def frame_id(_frame, _sensor_id, _tag_id):
        return "body"


class FakeGeometry:
    def resolve(
        self,
        object_id,
        routine_id,
        reference_view_id,
        pixel,
        approach_mode,
        target_surface_distance_m,
        aligned_preapproach_distance_m,
        hand_to_probe_pose,
    ):
        target = reference_view_surface_target.ReferenceSurfaceTarget(
            surface_point_object=Vector3Data(x=0.4, y=0.1, z=0.2),
            outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
            target_pose_object=pose(0.4 + target_surface_distance_m),
            aligned_preapproach_pose_object=pose(
                0.4 + aligned_preapproach_distance_m
            ),
            target_surface_distance_m=target_surface_distance_m,
            aligned_preapproach_distance_m=(
                aligned_preapproach_distance_m
            ),
            direction_source=approach_mode,
        )
        return SimpleNamespace(
            object_id=object_id,
            routine_id=routine_id,
            reference_view_id=reference_view_id,
            pixel=pixel,
            probe_setup=initialize_reference_probe_setup(
                target,
                hand_to_probe_pose,
            ),
        )


def pose(x=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=0.0, z=0.0),
        orientation=QuaternionData.identity(),
    )


def roll_quaternion(degrees):
    half_angle = math.radians(degrees) * 0.5
    return QuaternionData(
        x=math.sin(half_angle),
        y=0.0,
        z=0.0,
        w=math.cos(half_angle),
    )


def coordinator(tmp_path):
    command_controller = FakeCommandController()
    shared = SetupCoordinator(command_controller)
    references = multi_reference_view_repository.MultiReferenceViewRepository(
        tmp_path / "objects"
    )
    sensors = SensorRepository(
        tmp_path / "sensors",
        tmp_path / "retired_sensors",
    )
    sensors.create(SensorDefinition(
        sensor_id="hall_probe",
        display_name="Hall probe",
        hand_to_probe=PoseData.identity(),
    ))
    motion_state = FakeMotionStateSource()
    probe = ProbeSetupCoordinator(
        setup_coordinator=shared,
        reference_repository=references,
        sensor_repository=sensors,
        geometry=FakeGeometry(),
        motion_state_source=motion_state,
        motion_command_factory=FakeMotionCommandFactory(),
    )
    return probe, command_controller


def approve_all(probe, command_controller, state):
    probe.motion_state_source.pose = pose(0.8)
    state = probe.begin_refinement(state.context)
    state = probe.approve_safe_pose(state.context)

    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(
            kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.10,
        ),
    )
    probe.submit_motion(operation)
    probe.motion_state_source.pose = pose(0.6)
    command_controller.succeed(operation.request)
    context = probe.context(state.context.context_id, "probe-ui")
    state = probe.approve_aligned_pose(context)

    draft = probe._drafts[state.context.context_id]
    draft.refinement.motion_states[RefinementStage.PROBE] = (
        RefinementMotionState.REACHED
    )
    probe.motion_state_source.pose = pose(0.5)
    return probe.approve_probe_pose(state.context)


def create_selected_routine(probe, context):
    state = probe.create_object(
        context,
        "motor",
        "Motor",
        7,
        "36h11",
    )
    state = probe.create_routine(
        state.context,
        "motor",
        "magnetic_scan",
        "Magnetic scan",
        "hall_probe",
    )
    definition = probe.object_repository.load("motor")
    routine = definition.get_routine("magnetic_scan")
    stored_routine = replace(
        routine,
        reference_views=[ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_depth",
            reference_dataset_path=(
                "reference_datasets/magnetic_scan/set/slot1_hand"
            ),
            view_id="slot1_hand",
            camera_id="hand",
            slot_index=0,
        )],
    )
    probe.object_repository.save(replace(
        definition,
        routines=[stored_routine],
    ))
    return state


def test_repository_transactions_do_not_enter_physical_command_lane(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    context = probe.open_context("probe-ui").context

    state = create_selected_routine(probe, context)

    assert state.selected_object_id == "motor"
    assert state.selected_routine_id == "magnetic_scan"
    assert command_controller.submitted == []


def test_probe_motion_uses_single_non_recordable_command_lane(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    probe.motion_state_source.pose = pose(0.8)
    state = probe.begin_refinement(state.context)
    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(
            kind=ProbeMotionKind.MOVE_SAFE_APPROACH,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.10,
        ),
    )

    assert operation.request.origin is CommandOrigin.PROBE_SETUP
    assert (
        operation.request.recording_policy
        is RecordingPolicy.EXCLUDE
    )
    assert operation.request.context_id == state.context.context_id
    assert (
        operation.request.command.command_id
        is CommandID.MOVE_ARM_TO_TAG
    )
    with pytest.raises(RuntimeError, match="active motion"):
        probe.prepare_motion(
            state.context,
            ProbeMotionRequest(
                kind=ProbeMotionKind.MOVE_SAFE_APPROACH
            ),
        )

    probe.submit_motion(operation)
    probe.motion_state_source.pose = state.refinement.candidate_pose(
        RefinementStage.SAFE_APPROACH
    )
    command_controller.succeed(operation.request)
    current = probe.context(state.context.context_id, "probe-ui")
    completed = probe.snapshot(current)

    assert len(command_controller.submitted) == 1
    assert completed.context.revision > state.context.revision
    assert (
        completed.refinement.motion_states[RefinementStage.SAFE_APPROACH]
        is RefinementMotionState.REACHED
    )


def test_alignment_motion_levels_hand_against_gravity_not_tag_roll(tmp_path):
    probe, _ = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    probe.motion_state_source.pose = pose(0.8)
    probe.motion_state_source.gravity_object_pose = PoseData(
        position=Vector3Data.zero(),
        orientation=roll_quaternion(180.0),
    )
    state = probe.begin_refinement(state.context)

    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(
            kind=ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.10,
        ),
    )

    target = probe._drafts[state.context.context_id].refinement.pending_motion
    target = target.target_pose_object
    gravity_orientation = multiply_quaternions(
        probe.motion_state_source.gravity_object_pose.orientation,
        target.orientation,
    )
    roll, _pitch, _yaw = quaternion_to_rpy(gravity_orientation)
    inward = rotate_vector(
        target.orientation,
        Vector3Data(x=1.0, y=0.0, z=0.0),
    )

    assert operation.request.command.command_id is CommandID.MOVE_ARM_TO_TAG
    assert abs(roll) < 1e-6
    assert inward.x == pytest.approx(1.0)
    assert inward.y == pytest.approx(0.0)
    assert inward.z == pytest.approx(0.0)


def test_probe_motion_cancellation_keeps_context_state_correlated(tmp_path):
    probe, _ = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    state = probe.begin_refinement(state.context)
    operation = probe.prepare_motion(
        state.context,
        ProbeMotionRequest(kind=ProbeMotionKind.MOVE_SAFE_APPROACH),
    )
    statuses = []
    probe.add_motion_status_listener(statuses.append)
    probe.submit_motion(operation)

    probe.cancel_motion(state.context, operation.request_id)

    assert statuses[-1].request_id == operation.request_id
    assert statuses[-1].state is CommandControllerState.CANCELLED
    assert (
        statuses[-1].snapshot.refinement.motion_states[
            RefinementStage.SAFE_APPROACH
        ]
        is RefinementMotionState.FAILED
    )


def test_selected_definition_metadata_is_server_owned(tmp_path):
    probe, _ = coordinator(tmp_path)
    context = probe.open_context("probe-ui").context
    selected = create_selected_routine(probe, context)

    assert selected.selected_reference_tag_id == 7
    assert selected.selected_reference_tag_family == "36h11"
    assert selected.selected_sensor_id == "hall_probe"

    object_only = probe.select_object(selected.context, "motor")

    assert object_only.selected_object_id == "motor"
    assert object_only.selected_routine_id == ""
    assert object_only.selected_reference_tag_id == 7
    assert object_only.selected_sensor_id == ""


def test_geometry_and_approvals_are_owned_by_context(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    context = probe.open_context("probe-ui").context
    state = create_selected_routine(probe, context)
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    state = approve_all(probe, command_controller, state)

    assert state.reference_pixel == ImagePoint(u=20, v=30)
    assert state.setup.safe_approach_approved
    assert state.setup.surface_alignment_approved
    assert state.setup.probe_pose_approved


def test_distance_change_retains_only_safe_and_aligned_progress(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    state = approve_all(probe, command_controller, state)

    updated = probe.update_geometry(
        state.context,
        "surface_fit",
        0.12,
        0.24,
    )

    assert updated.setup.safe_approach_approved
    assert updated.setup.surface_alignment_approved
    assert not updated.setup.probe_pose_approved
    assert updated.setup.safe_approach_pose_object == pose(0.8)
    assert (
        updated.setup.aligned_preapproach_pose_object
        == state.setup.aligned_preapproach_pose_object
    )


def test_save_is_one_atomic_repository_transaction(tmp_path):
    probe, command_controller = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    state = approve_all(probe, command_controller, state)

    saved = probe.save_probe_point(
        state.context,
        "bearing_front",
        "Front bearing",
        0.01,
        0.10,
        1.0,
    )

    routine = probe.object_repository.load("motor").get_routine(
        "magnetic_scan"
    )
    point = routine.get_probe_point("bearing_front")
    assert point.reference_pixel == ImagePoint(u=20, v=30)
    assert point.reference_view_id == "slot1_hand"
    assert saved.probe_point_ids == ("bearing_front",)
    assert not saved.dirty


def test_unapproved_probe_point_cannot_be_saved(tmp_path):
    probe, _ = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )

    with pytest.raises(ValueError, match="All three"):
        probe.save_probe_point(
            state.context,
            "bearing_front",
            "Front bearing",
            0.01,
            0.10,
            1.0,
        )


def test_snapshot_geometry_is_isolated_from_internal_draft(tmp_path):
    probe, _ = coordinator(tmp_path)
    state = create_selected_routine(
        probe,
        probe.open_context("probe-ui").context,
    )
    state = probe.select_reference_pixel(
        state.context,
        "slot1_hand",
        ImagePoint(u=20, v=30),
        "surface_fit",
        0.10,
        0.20,
    )
    state.setup.safe_approach_pose_object.position.x = 99.0

    current = probe.snapshot(state.context)

    assert current.setup.safe_approach_pose_object.position.x != 99.0


def test_context_ownership_and_stale_revisions_are_enforced(tmp_path):
    probe, _ = coordinator(tmp_path)
    opened = probe.open_context("probe-ui")
    updated = probe.create_object(
        opened.context,
        "motor",
        "Motor",
        7,
        "36h11",
    )

    with pytest.raises(ValueError, match="does not own"):
        probe.context(updated.context.context_id, "other-ui")
    with pytest.raises(StaleSetupContext):
        probe.refresh(opened.context)


def test_pure_authoring_core_has_no_execution_command_dependency():
    package = Path(__file__).parents[1] / "fault_detector_spot"
    paths = (
        package / "inspection/setup/probe_setup_context.py",
        package / "inspection/setup/probe_setup_geometry.py",
        package / "application/coordinators/probe_setup_coordinator.py",
        package / "application/api/probe_setup_api.py",
    )
    source = "\n".join(path.read_text(encoding="utf-8") for path in paths)

    assert "CommandID" not in source
    assert "BasicCommand" not in source
    assert "ComplexCommand" not in source
    assert "ActionServer" not in source
    assert "command_controller.submit" not in source


def test_motion_transport_is_separate_from_authoring_transport():
    package = Path(__file__).parents[1] / "fault_detector_spot"
    authoring = (
        package / "application/api/probe_setup_api.py"
    ).read_text(encoding="utf-8")
    motion = (
        package / "application/api/probe_setup_motion_api.py"
    ).read_text(encoding="utf-8")

    assert "ActionServer" not in authoring
    assert "create_service" not in motion
    assert "ActionServer" in motion
