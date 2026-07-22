"""Tests for approach-direction selection in inspection controls."""

import math
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
from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.reference_view_approach_direction import (
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_FALLBACK,
    APPROACH_SOURCE_TAG_X_SELECTED,
)


class FakePublisher:
    """Accept commands without affecting geometry tests."""

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


def make_reference_view(frame_id="hand_color_image_sensor"):
    """Orient object-frame +X toward camera-frame negative Z."""
    half_sqrt = math.sqrt(0.5)
    return ReferenceView(
        controlled_frame_pose_object=PoseData(
            position=Vector3Data.zero(),
            orientation=QuaternionData(
                x=0.0,
                y=-half_sqrt,
                z=0.0,
                w=half_sqrt,
            ),
        ),
        controlled_frame=frame_id,
    )


def configure_reference(controls, values, frame_id="hand_color_image_sensor"):
    """Inject saved depth, calibration, and reference-view pose."""
    depth, camera_info = make_depth_and_camera(values)
    controls._reference_rgb_size = (depth.width, depth.height)
    controls._reference_depth_image = depth
    controls._reference_camera_info = camera_info
    controls._reference_view = make_reference_view(frame_id)


def test_automatic_mode_uses_surface_fit(application, tmp_path):
    """A valid local plane is the preferred outward direction source."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)

    controls._project_selected_reference_pixel(5, 5)

    result = controls.selected_approach_direction
    assert result is not None
    assert result.source == APPROACH_SOURCE_SURFACE_FIT
    assert controls.reference_approach_source_value_label.text() == (
        "Surface fit"
    )
    assert controls.reference_approach_z_value_label.text() == "-1.000"
    assert controls.reference_approach_status_label.text() == "Ready"


def test_automatic_mode_falls_back_to_tag_x(application, tmp_path):
    """An uneven or sparse surface still yields an outward direction."""
    values = [float("nan")] * 121
    values[5 * 11 + 5] = 1.0
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, values)

    controls._project_selected_reference_pixel(5, 5)

    result = controls.selected_approach_direction
    assert controls.selected_surface_normal is None
    assert result is not None
    assert result.source == APPROACH_SOURCE_TAG_X_FALLBACK
    assert controls.reference_approach_source_value_label.text() == (
        "Tag +X fallback"
    )
    assert controls.reference_approach_z_value_label.text() == "-1.000"
    assert controls.reference_approach_status_label.text() == "Ready"
    assert "Too few consistent depth" in (
        controls.reference_approach_status_label.toolTip()
    )


def test_surface_fit_only_keeps_failed_fit_unavailable(
    application,
    tmp_path,
):
    """The user can explicitly disable the automatic tag fallback."""
    values = [float("nan")] * 121
    values[5 * 11 + 5] = 1.0
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, values)
    index = controls.reference_approach_mode_dropdown.findData(
        APPROACH_MODE_SURFACE_FIT
    )
    controls.reference_approach_mode_dropdown.setCurrentIndex(index)

    controls._project_selected_reference_pixel(5, 5)

    assert controls.selected_approach_direction is None
    assert controls.reference_approach_status_label.text() == "Unavailable"
    assert "Surface-fit approach" in (
        controls.reference_approach_status_label.toolTip()
    )


def test_tag_x_can_be_selected_directly(application, tmp_path):
    """The user may choose tag-relative +X even with a valid plane."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    controls._project_selected_reference_pixel(5, 5)
    index = controls.reference_approach_mode_dropdown.findData(
        APPROACH_MODE_TAG_X
    )

    controls.reference_approach_mode_dropdown.setCurrentIndex(index)

    result = controls.selected_approach_direction
    assert result is not None
    assert result.source == APPROACH_SOURCE_TAG_X_SELECTED
    assert controls.reference_approach_source_value_label.text() == (
        "Tag +X selected"
    )
    assert controls.reference_approach_z_value_label.text() == "-1.000"


def test_reference_frame_mismatch_is_reported(application, tmp_path):
    """A saved tag orientation cannot be used in the wrong camera frame."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121, frame_id="other_camera")

    controls._project_selected_reference_pixel(5, 5)

    assert controls.selected_approach_direction is None
    assert controls.reference_approach_status_label.text() == (
        "Frame mismatch"
    )


def test_approach_readouts_remain_fixed(application, tmp_path):
    """Changing direction sources cannot resize the reference preview."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    labels = (
        controls.reference_approach_source_value_label,
        controls.reference_approach_status_label,
        controls.reference_approach_x_value_label,
        controls.reference_approach_y_value_label,
        controls.reference_approach_z_value_label,
    )
    widths = [
        (label.minimumWidth(), label.maximumWidth())
        for label in labels
    ]
    panel_height = controls.reference_point_panel.height()

    controls._project_selected_reference_pixel(5, 5)

    assert all(minimum == maximum for minimum, maximum in widths)
    assert controls.reference_point_panel.minimumHeight() == panel_height
    assert controls.reference_point_panel.maximumHeight() == panel_height
