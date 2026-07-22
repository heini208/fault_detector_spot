"""Tests for resolution-aware reference geometry in the UI."""

import os
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.models import PoseData, ReferenceView


class FakePublisher:
    """Accept commands without side effects."""

    def publish(self, message):
        """Discard one command."""


class FakeUI:
    """Provide the parent contract required by inspection controls."""

    def __init__(self, object_root):
        """Create isolated UI dependencies."""
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root

    def build_basic_command(self, command_id):
        """Build a minimal command message."""
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    """Provide the Qt application used by the controls."""
    return QApplication.instance() or QApplication([])


def configure_reference(controls):
    """Inject a 22x22 RGB view with 11x11 registered depth."""
    depth = Image()
    depth.header.frame_id = "hand_color_image_sensor"
    depth.width = 11
    depth.height = 11
    depth.encoding = "32FC1"
    depth.step = 44
    depth.data = b"".join(struct.pack("<f", 1.0) for _ in range(121))

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 11
    camera_info.height = 11
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

    controls._reference_rgb_size = (22, 22)
    controls._reference_depth_image = depth
    controls._reference_camera_info = camera_info
    controls._reference_view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
    )


def test_controls_map_rgb_selection_before_normal_fit(
    application,
    tmp_path,
):
    """The UI exposes mapped depth coordinates and a valid flat normal."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls)

    controls._project_selected_reference_pixel(10, 10)

    point = controls.selected_surface_point
    normal = controls.selected_surface_normal
    assert point is not None
    assert point.mapped_pixel.u == 5
    assert point.mapped_pixel.v == 5
    assert normal is not None
    assert controls.reference_depth_pixel_value_label.text() == "u=5, v=5"
    assert "22x22" in controls.reference_depth_pixel_value_label.toolTip()
    assert controls.reference_normal_status_label.text() == "Ready"
    assert controls.reference_normal_z_value_label.text() == "-1.000"
