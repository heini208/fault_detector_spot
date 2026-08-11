"""Tests for displaying reference surface-normal estimates in the UI."""

import os
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)


class FakePublisher:
    """Accept commands without affecting normal-estimation tests."""

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
        self.visible_tags = {}

    def build_basic_command(self, command_id):
        """Build a minimal command header."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application required by controls."""
    return QApplication.instance() or QApplication([])


def make_depth_and_camera(values, width=11, height=11):
    """Create one registered floating-point depth dataset."""
    depth = Image()
    depth.header.frame_id = "hand_color_image_sensor"
    depth.width = width
    depth.height = height
    depth.encoding = "32FC1"
    depth.step = width * 4
    depth.data = b"".join(struct.pack("<f", value) for value in values)

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = width
    camera_info.height = height
    camera_info.k = [
        100.0,
        0.0,
        5.0,
        0.0,
        100.0,
        5.0,
        0.0,
        0.0,
        1.0,
    ]
    return depth, camera_info


def configure_reference_depth(controls, values):
    """Inject a synchronized reference depth dataset into the controls."""
    depth, camera_info = make_depth_and_camera(values)
    controls._reference_rgb_size = (depth.width, depth.height)
    controls._reference_depth_image = depth
    controls._reference_camera_info = camera_info
    controls._reference_rgb_camera_info = camera_info


def test_controls_expose_surface_normal_and_quality(application, tmp_path):
    """A valid plane populates stable normal and fit readouts."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference_depth(controls, [1.0] * 121)

    controls._project_selected_reference_pixel(5, 5)

    result = controls.selected_surface_normal
    assert result is not None
    assert result.normal_camera.z == pytest.approx(-1.0)
    assert controls.reference_normal_x_value_label.text() == "0.000"
    assert controls.reference_normal_y_value_label.text() == "0.000"
    assert controls.reference_normal_z_value_label.text() == "-1.000"
    assert int(controls.reference_normal_samples_value_label.text()) >= 40
    assert controls.reference_normal_rmse_value_label.text() == "0.0000"
    assert controls.reference_normal_status_label.text() == "Ready"


def test_normal_failure_preserves_projected_surface_point(
    application,
    tmp_path,
):
    """A valid point remains usable when its plane fit is underconstrained."""
    values = [float("nan")] * 121
    values[5 * 11 + 5] = 1.0
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference_depth(controls, values)

    controls._project_selected_reference_pixel(5, 5)

    assert controls.selected_surface_point is not None
    assert controls.reference_projection_status_label.text() == "Ready"
    assert controls.selected_surface_normal is None
    assert controls.reference_normal_status_label.text() == "Unavailable"
    assert "Too few consistent depth" in (
        controls.reference_normal_status_label.toolTip()
    )


def test_clearing_point_clears_normal_state(application, tmp_path):
    """Normal state cannot outlive its projected reference point."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference_depth(controls, [1.0] * 121)
    controls._project_selected_reference_pixel(5, 5)

    controls._clear_selected_surface_point()

    assert controls.selected_surface_point is None
    assert controls.selected_surface_normal is None
    assert controls.reference_normal_z_value_label.text() == "—"
    assert controls.reference_normal_status_label.text() == "No point"


def test_normal_readout_widths_remain_fixed(application, tmp_path):
    """Changing normal values keeps the target-tab geometry stable."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference_depth(controls, [1.0] * 121)
    labels = (
        controls.reference_normal_x_value_label,
        controls.reference_normal_y_value_label,
        controls.reference_normal_z_value_label,
        controls.reference_normal_samples_value_label,
        controls.reference_normal_rmse_value_label,
        controls.reference_normal_status_label,
    )
    widths = [
        (label.minimumWidth(), label.maximumWidth())
        for label in labels
    ]
    panel_height = controls.reference_point_panel.height()

    controls._project_selected_reference_pixel(5, 5)

    assert all(minimum == maximum for minimum, maximum in widths)
    assert controls.reference_point_panel.height() == panel_height
    assert controls.reference_point_panel.minimumHeight() <= panel_height
    assert controls.reference_point_panel.maximumHeight() >= panel_height
