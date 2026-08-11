"""Tests for surface-orientation and target-pose inspection controls."""

import math
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
from fault_detector_spot.inspection.data.models import (
    PoseData,
    QuaternionData,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_view_approach_direction import (
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_SELECTED,
)


class FakePublisher:
    def publish(self, message):
        pass


class FakeUI:
    def __init__(self, object_root):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def make_depth_and_camera(values, width=11, height=11):
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
    depth, camera_info = make_depth_and_camera(values)
    controls._reference_rgb_size = (depth.width, depth.height)
    controls._reference_depth_image = depth
    controls._reference_camera_info = camera_info
    controls._reference_rgb_camera_info = camera_info
    controls._reference_view = make_reference_view(frame_id)


def test_automatic_mode_generates_surface_target(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)

    controls._project_selected_reference_pixel(5, 5)

    direction = controls.selected_approach_direction
    target = controls.selected_surface_target
    assert direction is not None
    assert direction.source == APPROACH_SOURCE_SURFACE_FIT
    assert target is not None
    assert controls.reference_approach_source_value_label.text() == (
        "Surface fit"
    )
    assert controls.reference_target_status_label.text() == "Ready"
    assert controls.reference_target_roll_value_label.text() == "0.0"
    assert controls.reference_target_pitch_value_label.text() == "0.0"
    assert controls.reference_target_yaw_value_label.text() == "-180.0"


def test_automatic_mode_rejects_uncalibrated_tag_fallback(
    application,
    tmp_path,
):
    values = [float("nan")] * 121
    values[5 * 11 + 5] = 1.0
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, values)

    controls._project_selected_reference_pixel(5, 5)

    assert controls.selected_surface_normal is None
    assert controls.selected_approach_direction is None
    assert controls.selected_surface_target is None
    assert controls.reference_approach_status_label.text() == "Unavailable"
    assert "Too few consistent depth" in (
        controls.reference_approach_status_label.toolTip()
    )


def test_surface_fit_only_keeps_target_unavailable(application, tmp_path):
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
    assert controls.selected_surface_target is None
    assert controls.reference_approach_status_label.text() == "Unavailable"


def test_tag_x_selected_generates_tag_aligned_target(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    controls._project_selected_reference_pixel(5, 5)
    index = controls.reference_approach_mode_dropdown.findData(
        APPROACH_MODE_TAG_X
    )

    controls.reference_approach_mode_dropdown.setCurrentIndex(index)

    direction = controls.selected_approach_direction
    target = controls.selected_surface_target
    assert direction is not None
    assert direction.source == APPROACH_SOURCE_TAG_X_SELECTED
    assert target is not None
    assert controls.reference_target_roll_value_label.text() == "0.0"
    assert controls.reference_target_pitch_value_label.text() == "0.0"
    assert controls.reference_target_yaw_value_label.text() == "-180.0"


def test_distance_change_recalculates_target(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    controls._project_selected_reference_pixel(5, 5)
    old_position = (
        controls.selected_surface_target.target_pose_object.position.x
    )

    controls.reference_target_distance_field.setText("0.08")
    controls.reference_preapproach_distance_field.setText("0.15")
    controls._handle_target_distance_changed()

    new_position = (
        controls.selected_surface_target.target_pose_object.position.x
    )
    assert new_position - old_position == pytest.approx(0.05)


def test_preapproach_field_uses_absolute_surface_distance(
    application,
    tmp_path,
):
    """The two UI distances remain independent absolute values."""
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    controls.reference_target_distance_field.setText("0.10")
    controls.reference_preapproach_distance_field.setText("0.15")

    controls._project_selected_reference_pixel(5, 5)

    target = controls.selected_surface_target
    assert target is not None
    assert target.target_surface_distance_m == pytest.approx(0.10)
    assert target.aligned_preapproach_distance_m == pytest.approx(
        0.15
    )


def test_reference_frame_mismatch_is_reported(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121, frame_id="other_camera")

    controls._project_selected_reference_pixel(5, 5)

    assert controls.selected_approach_direction is None
    assert controls.selected_surface_target is None
    assert controls.reference_approach_status_label.text() == (
        "Frame mismatch"
    )


def test_target_readouts_remain_fixed(application, tmp_path):
    controls = InspectionControls(FakeUI(tmp_path))
    configure_reference(controls, [1.0] * 121)
    labels = (
        controls.reference_approach_source_value_label,
        controls.reference_approach_status_label,
        controls.reference_target_status_label,
        controls.reference_target_x_value_label,
        controls.reference_target_y_value_label,
        controls.reference_target_z_value_label,
        controls.reference_target_roll_value_label,
        controls.reference_target_pitch_value_label,
        controls.reference_target_yaw_value_label,
        controls.reference_preapproach_x_value_label,
        controls.reference_preapproach_y_value_label,
        controls.reference_preapproach_z_value_label,
    )
    widths = [
        (label.minimumWidth(), label.maximumWidth())
        for label in labels
    ]

    controls._project_selected_reference_pixel(5, 5)

    assert all(minimum == maximum for minimum, maximum in widths)
    assert controls.reference_point_panel.minimumHeight() < (
        controls.reference_point_panel.maximumHeight()
    )
    assert controls.geometry_details_section.content_frame.isHidden()
