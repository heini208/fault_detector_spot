"""Tests for saving approved probe points from inspection controls."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
    ReferenceView,
)


class FakePublisher:
    """Discard compatibility command publications."""

    def publish(self, message):
        """Accept one compatibility command."""
        pass


class FakeUI:
    """Provide the UI dependencies used by inspection controls."""

    def __init__(self, object_root):
        """Initialize an isolated repository-backed UI."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.visible_tags = {}

    def build_basic_command(self, command_id):
        """Build the command header required by the controls."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by widgets."""
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application, tmp_path):
    """Create isolated inspection controls."""
    result = InspectionControls(FakeUI(tmp_path))
    result.show_warning = lambda title, message: None
    return result


def pose(x):
    """Create an identity pose with one translated coordinate."""
    result = PoseData.identity()
    result.position.x = x
    return result


def make_definition():
    """Create one routine with a persisted reference-view identity."""
    view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/magnetic_scan/set_1/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[InspectionRoutine(
            routine_id="magnetic_scan",
            display_name="Magnetic scan",
            sensor_id="bmm150",
            probe_frame="sensor_tip",
            reference_views=[view],
        )],
    )


def configure_ready_probe(controls):
    """Populate one fully approved transient probe setup."""
    definition = make_definition()
    controls.object_repository.create(definition)
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )
    controls._reference_view = (
        definition.get_routine("magnetic_scan").reference_views[0]
    )
    controls._selected_surface_point = SimpleNamespace(
        requested_pixel=ImagePoint(u=120, v=80)
    )
    controls._probe_setup = SimpleNamespace(
        surface_target=SimpleNamespace(
            target_surface_distance_m=0.03,
            aligned_preapproach_distance_m=0.08,
        ),
        safe_approach_pose_object=pose(0.30),
        probe_pose_object=pose(0.03),
        safe_approach_approved=True,
        surface_alignment_approved=True,
        probe_pose_approved=True,
    )
    controls.probe_point_id_field.setText("point_a")
    controls.probe_point_display_name_field.setText("Point A")
    controls._update_probe_setup_status_widgets()


def test_save_persists_the_approved_geometry_and_rgb_provenance(
    controls,
):
    """Save uses approved poses and the selected RGB pixel."""
    configure_ready_probe(controls)

    assert controls.save_probe_point_button.isEnabled()
    assert controls.handle_save_probe_point() is True

    stored = controls.object_repository.load("motor_a")
    point = stored.get_routine("magnetic_scan").probe_points[0]
    assert point.probe_point_id == "point_a"
    assert point.display_name == "Point A"
    assert point.safe_approach_pose_object.position.x == pytest.approx(
        0.30
    )
    assert point.probe_pose_object.position.x == pytest.approx(0.03)
    assert point.target_surface_distance_m == pytest.approx(0.03)
    assert point.preapproach_distance_m == pytest.approx(0.05)
    assert point.position_tolerance_m == pytest.approx(0.01)
    assert point.orientation_tolerance_rad == pytest.approx(0.087)
    assert point.measurement_duration_sec == pytest.approx(1.0)
    assert point.reference_pixel == ImagePoint(u=120, v=80)
    assert point.reference_view_id == "slot1_hand"


def test_save_requires_every_approval_and_valid_numeric_fields(controls):
    """Incomplete or invalid definitions keep saving disabled."""
    configure_ready_probe(controls)
    controls._probe_setup.probe_pose_approved = False
    controls._update_probe_setup_status_widgets()

    assert not controls.save_probe_point_button.isEnabled()
    assert controls.handle_save_probe_point() is False
    assert not controls.object_repository.load(
        "motor_a"
    ).get_routine("magnetic_scan").probe_points

    controls._probe_setup.probe_pose_approved = True
    controls.probe_position_tolerance_field.setText("nan")
    controls._update_probe_setup_status_widgets()

    assert not controls.save_probe_point_button.isEnabled()
