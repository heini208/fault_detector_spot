"""Tests for transient probe setup commands in inspection controls."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand, TagElement
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
    OrientationModes,
)
from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.live_surface_distance import (
    SurfaceDistanceSample,
)
from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.probe_refinement_session import (
    ProbeRefinementSession,
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.sensor_models import SensorDefinition
from fault_detector_spot.inspection.reference_probe_setup import (
    approve_surface_alignment_pose,
    initialize_reference_probe_setup,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    ReferenceSurfaceTarget,
)
from fault_detector_spot.inspection.reference_view_depth_projection import (
    ImageRegion,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeUI:
    def __init__(self, object_root):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}
        self.sensor_definitions = [
            SensorDefinition(
                sensor_id="bmm150",
                display_name="BMM150 Hall sensor",
                hand_to_probe=PoseData.identity(),
            )
        ]

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def pose(x=0.0, y=0.0, z=0.0):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=QuaternionData.identity(),
    )


def visible_tag(tag_id=7):
    tag = TagElement()
    tag.id = tag_id
    tag.pose.header.frame_id = "body"
    tag.pose.pose.position.x = 1.0
    tag.pose.pose.orientation.w = 1.0
    return tag


def configure_live_tag(controls, ui, tag_id=7):
    controls._selected_definition = SimpleNamespace(
        reference_tag=SimpleNamespace(tag_id=tag_id)
    )
    tag = visible_tag(tag_id)
    ui.visible_tags[tag_id] = tag
    return tag


def surface_target():
    return ReferenceSurfaceTarget(
        surface_point_object=Vector3Data.zero(),
        outward_direction_object=Vector3Data(x=1.0, y=0.0, z=0.0),
        target_pose_object=pose(x=0.03),
        aligned_preapproach_pose_object=pose(x=0.08),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.08,
        direction_source="surface_fit",
    )


def distance_sample(distance_m, stamp_seconds):
    return SurfaceDistanceSample(
        distance_m=distance_m,
        stamp_seconds=stamp_seconds,
        frame_id="hand_depth",
        sample_count=30,
        valid_pixel_ratio=0.8,
        spread_m=0.001,
        source_region=ImageRegion(x=0, y=0, width=3, height=3),
    )


def configure_session(controls, setup, stage):
    controls._calculated_probe_setup = initialize_reference_probe_setup(
        surface_target()
    )
    controls._probe_setup = setup
    controls._refinement_session = ProbeRefinementSession.create(
        controls._calculated_probe_setup,
        setup,
    )
    controls._refinement_session.active_stage = stage
    controls._refinement_workflow_active = True
    return controls._refinement_session


def test_probe_command_converts_sensor_tip_pose_to_hand_pose(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    configure_live_tag(controls, ui)
    controls._hand_to_probe_pose = lambda: pose(x=0.10)

    command = controls._build_probe_pose_command(pose(x=0.20))

    assert command.command.command_id == "move_to_tag"
    assert command.orientation_mode == (
        OrientationModes.CUSTOM_ORIENTATION.value
    )
    assert command.offset.header.frame_id == "body"
    assert command.offset.pose.position.x == pytest.approx(0.10)
    assert command.offset.pose.position.y == pytest.approx(0.0)
    assert command.offset.pose.position.z == pytest.approx(0.0)
    assert command.offset.pose.orientation.w == pytest.approx(1.0)


def test_current_probe_pose_is_expressed_relative_to_live_tag(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    configure_live_tag(controls, ui)
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("bmm150")
    )
    controls._lookup_pose = lambda target, source: pose(x=1.30)

    current = controls._current_probe_pose_object()

    assert current.position.x == pytest.approx(0.30)
    assert current.position.y == pytest.approx(0.0)
    assert current.position.z == pytest.approx(0.0)


def test_starting_wizard_never_publishes_a_robot_command(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    setup = initialize_reference_probe_setup(surface_target())
    controls._calculated_probe_setup = setup
    controls._probe_setup = setup
    current = pose(x=0.42, y=0.10, z=0.30)
    controls._current_probe_pose_object = lambda: current

    assert controls.handle_start_probe_refinement() is True

    assert ui.complex_command_publisher.messages == []
    assert controls._refinement_session.active_stage == (
        RefinementStage.SAFE_APPROACH
    )
    assert controls._refinement_session.candidate_pose(
        RefinementStage.SAFE_APPROACH
    ) == current
    assert controls._refinement_session.motion_states[
        RefinementStage.SAFE_APPROACH
    ] == RefinementMotionState.REACHED
    assert not controls._refinement_session.stage_is_approved(
        RefinementStage.SAFE_APPROACH
    )
    assert controls.refinement_dialog.isVisible()
    controls.refinement_dialog.close()
    assert not controls._refinement_workflow_active


def test_direct_probe_motion_is_disabled(application, tmp_path):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    controls.show_warning = lambda title, message: None

    assert controls.handle_move_to_probe_pose() is False

    assert ui.complex_command_publisher.messages == []
    assert "Direct probe-pose movement is disabled" in (
        controls.reference_setup_status_label.toolTip()
    )


def test_alignment_refinement_uses_tangent_axes_and_has_no_axial_buttons(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.show_warning = lambda title, message: None
    setup = initialize_reference_probe_setup(surface_target())
    setup = approve_surface_alignment_pose(setup, pose(x=0.08))
    session = configure_session(
        controls,
        setup,
        RefinementStage.ALIGNMENT,
    )
    session.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    session.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    adjustments = []
    controls._send_refinement_relative_motion = (
        lambda stage, label, translation, pitch, yaw:
        adjustments.append(
            (stage, translation, pitch, yaw, label)
        ) or True
    )

    assert controls.handle_refine_pose("alignment", "up") is True

    assert adjustments[0][0] == RefinementStage.ALIGNMENT
    assert adjustments[0][1].z == pytest.approx(0.01)
    assert adjustments[0][1].x == pytest.approx(0.0)
    assert "front" not in controls.refinement_buttons["alignment"]
    assert "back" not in controls.refinement_buttons["alignment"]
    assert controls.handle_refine_pose("probe", "up") is False


def test_relative_refinement_completion_does_not_query_tag_pose(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    controls.show_warning = lambda title, message: None
    setup = initialize_reference_probe_setup(surface_target())
    session = configure_session(
        controls,
        setup,
        RefinementStage.SAFE_APPROACH,
    )
    session.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("bmm150")
    )
    controls._current_probe_pose_object = lambda: (_ for _ in ()).throw(
        AssertionError("relative completion queried the tag pose")
    )

    assert controls.handle_refine_pose("approach", "up") is True

    command = ui.complex_command_publisher.messages[-1]
    pending = session.pending_motion
    assert command.command.command_id == CommandID.MOVE_ARM_RELATIVE
    assert command.offset.header.frame_id == "bmm150_probe"
    assert command.offset.pose.position.z == pytest.approx(0.01)
    assert not pending.verify_achieved_pose

    controls._complete_pending_refinement_motion(pending.request_id)

    assert session.pending_motion is None
    assert session.motion_states[RefinementStage.SAFE_APPROACH] == (
        RefinementMotionState.REACHED
    )


def test_refinement_frame_selector_defaults_to_sensor_and_derives_frames(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("bmm150")
    )
    controls._selected_definition = SimpleNamespace(
        reference_tag=SimpleNamespace(tag_id=7)
    )

    assert controls.refine_frame_dropdown.currentData() == "sensor"
    assert controls._selected_refinement_frame_id() == "bmm150_probe"

    expected = {
        "hand": "hand",
        "tag": "filtered_fiducial_7",
        "body": "body",
        "map": "map",
    }
    for selection, frame_id in expected.items():
        controls.refine_frame_dropdown.setCurrentIndex(
            controls.refine_frame_dropdown.findData(selection)
        )
        assert controls._selected_refinement_frame_id() == frame_id


def test_surface_distance_test_commands_only_one_bounded_axis_step(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.show_warning = lambda title, message: None
    setup = initialize_reference_probe_setup(surface_target())
    approved = approve_surface_alignment_pose(
        setup,
        pose(x=0.08),
    )
    session = configure_session(
        controls,
        approved,
        RefinementStage.PROBE,
    )
    session.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    session.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    controls._current_probe_pose_object = lambda: pose(x=0.08)
    controls._measure_live_surface_distance_samples = lambda: [
        distance_sample(0.079, 10.0),
        distance_sample(0.080, 10.1),
        distance_sample(0.081, 10.2),
    ]
    targets = []
    controls._send_refinement_motion = (
        lambda stage, label, target, **kwargs:
        targets.append((stage, target, label, kwargs)) or True
    )

    assert controls.handle_test_surface_distance() is True

    assert targets[0][0] == RefinementStage.PROBE
    assert targets[0][1].position.x == pytest.approx(0.07)
    assert "inward correction" in targets[0][2]
    assert targets[0][3]["axial_correction_m"] == pytest.approx(0.01)
    assert controls.surface_distance_delta_value_label.text() == "0.0500"


def test_surface_distance_verification_requires_all_three_frames(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.show_warning = lambda title, message: None
    setup = initialize_reference_probe_setup(surface_target())
    approved = approve_surface_alignment_pose(setup, pose(x=0.08))
    session = configure_session(
        controls,
        approved,
        RefinementStage.PROBE,
    )
    session.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    session.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    controls._current_probe_pose_object = lambda: pose(x=0.03)
    controls._measure_live_surface_distance_samples = lambda: [
        distance_sample(0.029, 10.0),
        distance_sample(0.030, 10.1),
        distance_sample(0.031, 10.2),
    ]

    assert controls.handle_test_surface_distance() is True

    assert session.surface_distance_verified
    assert session.motion_states[RefinementStage.PROBE] == (
        RefinementMotionState.REACHED
    )
    assert "Surface Distance Verified" in (
        controls.surface_distance_test_status_label.text()
    )


def test_missing_samples_before_first_inward_move_can_be_retried(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls.show_warning = lambda title, message: None
    setup = initialize_reference_probe_setup(surface_target())
    approved = approve_surface_alignment_pose(setup, pose(x=0.08))
    session = configure_session(
        controls,
        approved,
        RefinementStage.PROBE,
    )
    session.motion_states[RefinementStage.SAFE_APPROACH] = (
        RefinementMotionState.REACHED
    )
    session.motion_states[RefinementStage.ALIGNMENT] = (
        RefinementMotionState.REACHED
    )
    controls._measure_live_surface_distance_samples = lambda: (_ for _ in ()).throw(
        ValueError("sampling window is not ready")
    )

    assert controls.handle_test_surface_distance() is False

    assert not session.recovery_required
    assert not controls._distance_failure_requires_retraction
