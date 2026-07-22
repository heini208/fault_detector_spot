"""Tests for displaying selected reference surface points in the UI."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import CameraInfo, Image

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
from fault_detector_spot.inspection.object_repository import ObjectRepository


class FakePublisher:
    """Accept commands without affecting projection tests."""

    def publish(self, message):
        """Discard one command."""


class FakeUI:
    """Provide the parent contract used by inspection controls."""

    def __init__(self, object_root):
        """Create isolated UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root

    def build_basic_command(self, command_id):
        """Build a minimal command header."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by controls."""
    return QApplication.instance() or QApplication([])


def make_definition():
    """Create one inspection routine ready for a reference capture."""
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(tag_id=23, tag_family="36h11"),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id="bmm150",
                probe_frame="sensor_tip",
            )
        ],
    )


def save_dataset(root, depth_values):
    """Persist a small registered reference dataset."""
    repository = ObjectRepository(root)
    repository.create(make_definition())
    rgb = Image()
    rgb.header.frame_id = "hand_color_image_sensor"
    rgb.header.stamp.sec = 10
    rgb.header.stamp.nanosec = 200_000_000
    rgb.width = 2
    rgb.height = 1
    rgb.encoding = "rgb8"
    rgb.step = 6
    rgb.data = bytes([255, 0, 0, 0, 255, 0])

    depth = Image()
    depth.header.frame_id = "hand_color_image_sensor"
    depth.header.stamp.sec = 10
    depth.header.stamp.nanosec = 210_000_000
    depth.width = 2
    depth.height = 1
    depth.encoding = "16UC1"
    depth.step = 4
    depth.data = b"".join(
        int(value).to_bytes(2, byteorder="little")
        for value in depth_values
    )

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 2
    camera_info.height = 1
    camera_info.k = [
        100.0,
        0.0,
        0.0,
        0.0,
        100.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_color_image_sensor",
        ),
        rgb,
        depth,
        camera_info,
    )


def select_routine(controls):
    """Select the single persisted object and routine."""
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )


def test_selected_pixel_exposes_projected_surface_point(
    application,
    tmp_path,
):
    """The selected pixel is resolved using the stored depth dataset."""
    save_dataset(tmp_path, [0, 1250])
    controls = InspectionControls(FakeUI(tmp_path))
    select_routine(controls)

    controls.reference_view_widget._set_selected_image_point(
        ImagePoint(u=1, v=0)
    )

    result = controls.selected_surface_point
    assert result is not None
    assert result.depth_m == pytest.approx(1.25)
    assert result.point_camera.x == pytest.approx(0.0125)
    assert controls.reference_surface_z_value_label.text() == "1.250"
    assert controls.reference_projection_status_label.text() == "Ready"


def test_invalid_depth_is_reported_without_accepting_a_point(
    application,
    tmp_path,
):
    """A selected pixel with no local depth remains visibly invalid."""
    save_dataset(tmp_path, [0, 0])
    controls = InspectionControls(FakeUI(tmp_path))
    select_routine(controls)

    controls.reference_view_widget._set_selected_image_point(
        ImagePoint(u=1, v=0)
    )

    assert controls.selected_surface_point is None
    assert controls.reference_projection_status_label.text() == (
        "Unavailable"
    )
    assert "No valid depth" in (
        controls.reference_projection_status_label.toolTip()
    )


def test_clearing_pixel_also_clears_surface_point(
    application,
    tmp_path,
):
    """Transient projection state cannot outlive its selected pixel."""
    save_dataset(tmp_path, [1000, 1250])
    controls = InspectionControls(FakeUI(tmp_path))
    select_routine(controls)
    controls.reference_view_widget._set_selected_image_point(
        ImagePoint(u=1, v=0)
    )

    controls.reference_view_widget.clear_selection()

    assert controls.selected_surface_point is None
    assert controls.reference_surface_z_value_label.text() == "—"
    assert controls.reference_projection_status_label.text() == "No point"


def test_projection_readout_geometry_is_stable(application, tmp_path):
    """Changing projection text cannot resize or move the image row."""
    save_dataset(tmp_path, [1000, 1250])
    controls = InspectionControls(FakeUI(tmp_path))
    select_routine(controls)

    panel_height = controls.reference_point_panel.height()
    widths = {
        label: (label.minimumWidth(), label.maximumWidth())
        for label in (
            controls.reference_pixel_value_label,
            controls.reference_surface_frame_value_label,
            controls.reference_surface_x_value_label,
            controls.reference_surface_y_value_label,
            controls.reference_surface_z_value_label,
            controls.reference_depth_pixel_value_label,
            controls.reference_projection_status_label,
        )
    }

    controls.reference_view_widget._set_selected_image_point(
        ImagePoint(u=1, v=0)
    )

    assert controls.reference_point_panel.minimumHeight() == panel_height
    assert controls.reference_point_panel.maximumHeight() == panel_height
    assert all(minimum == maximum for minimum, maximum in widths.values())
