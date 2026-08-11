"""Tests for loading captured reference images into inspection controls."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand, TagElement
from PyQt5.QtWidgets import QApplication, QLabel
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.data.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.inspection.data.object_repository import ObjectRepository
from fault_detector_spot.inspection.data.multi_reference_view_repository import (
    CapturedReferenceView,
    MultiReferenceViewRepository,
)


class FakePublisher:
    """Accept commands without affecting preview tests."""

    def publish(self, message):
        """Discard one published command."""


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


def make_definition() -> InspectionObject:
    """Create one object containing an uncaptured routine."""
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id="bmm150",
            )
        ],
    )


def make_reference_dataset():
    """Create one serializable RGB, depth, and camera-info dataset."""
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
    depth.data = bytes([100, 0, 100, 0])

    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = 2
    camera_info.height = 1
    camera_info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    tag = TagElement()
    tag.id = 23
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.orientation.w = 1.0
    return rgb, depth, camera_info, tag


def select_routine(controls):
    """Select the single persisted object and routine."""
    controls.refresh_saved_definitions()
    controls.saved_object_dropdown.setCurrentIndex(
        controls.saved_object_dropdown.findData("motor_a")
    )
    controls.saved_routine_dropdown.setCurrentIndex(
        controls.saved_routine_dropdown.findData("magnetic_scan")
    )


def test_captured_routine_loads_rgb_preview(application, tmp_path):
    """Selecting a captured routine displays its persisted RGB image."""
    repository = MultiReferenceViewRepository(tmp_path)
    repository.object_repository.create(make_definition())
    rgb, depth, camera_info, tag = make_reference_dataset()
    repository.save_reference_views(
        "motor_a",
        "magnetic_scan",
        [CapturedReferenceView(
            slot_index=0,
            camera_id="hand",
            reference_view=ReferenceView(
                controlled_frame_pose_object=PoseData.identity(),
                controlled_frame="hand_color_image_sensor",
            ),
            rgb_image=rgb,
            depth_image=depth,
            rgb_camera_info=camera_info,
            depth_camera_info=camera_info,
            reference_tag=tag,
            fixed_frame="odom",
        )],
    )
    controls = InspectionControls(FakeUI(tmp_path))

    select_routine(controls)

    assert controls.reference_view_widget.has_image is True
    assert "Reference view: captured" in (
        controls.reference_view_status_label.text()
    )


def test_uncaptured_routine_clears_preview(application, tmp_path):
    """A routine without teaching data shows a specific empty state."""
    ObjectRepository(tmp_path).create(make_definition())
    controls = InspectionControls(FakeUI(tmp_path))

    select_routine(controls)

    assert controls.reference_view_widget.has_image is False
    assert controls.reference_view_widget.text() == (
        "No reference view captured"
    )
    assert controls.reference_view_status_label.text() == (
        "Reference view: not captured"
    )


def test_missing_dataset_reports_load_failure(application, tmp_path):
    """A broken dataset path does not crash routine selection."""
    repository = ObjectRepository(tmp_path)
    definition = make_definition()
    definition.routines[0].reference_views = [ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/magnetic_scan/"
            "set_10_200000000/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )]
    repository.create(definition)
    controls = InspectionControls(FakeUI(tmp_path))

    select_routine(controls)

    assert controls.reference_view_widget.has_image is False
    assert controls.reference_view_widget.text() == (
        "Reference view unavailable"
    )
    assert "preview unavailable" in (
        controls.reference_view_status_label.text()
    )
