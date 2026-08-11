"""Tests for direct inspection-definition repository management."""

import os

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton

from fault_detector_spot.ui.controls.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.data.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ReferenceTag,
)
from fault_detector_spot.inspection.data.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.data.sensor_models import SensorDefinition


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class FakePublisher:
    """Record compatibility command publications."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeUI:
    """Provide the UI contract required by inspection controls."""

    def __init__(self, object_root):
        self.node = None
        self.status_label = QLabel()
        self.complex_command_publisher = FakePublisher()
        self.inspection_object_root = object_root
        self.sensor_definitions = [
            SensorDefinition(
                sensor_id="mlx90640",
                display_name="MLX90640 thermal sensor",
                hand_to_probe=PoseData.identity(),
            ),
            SensorDefinition(
                sensor_id="bmm150",
                display_name="BMM150 Hall sensor",
                hand_to_probe=PoseData.identity(),
            ),
        ]

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application, tmp_path):
    result = InspectionControls(FakeUI(tmp_path))
    result.show_warning = lambda title, message: None
    result.ask_question = lambda title, message: True
    return result


def save_object(
    controls,
    object_id="motor_a",
    routines=None,
):
    definition = InspectionObject(
        object_id=object_id,
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=list(routines or []),
    )
    ObjectRepository(
        controls.object_repository.root_dir
    ).create(definition)
    return definition


def select_object(controls, object_id):
    controls.refresh_saved_definitions()
    index = controls.saved_object_dropdown.findData(object_id)
    controls.saved_object_dropdown.setCurrentIndex(index)


def select_routine(controls, routine_id):
    index = controls.saved_routine_dropdown.findData(routine_id)
    controls.saved_routine_dropdown.setCurrentIndex(index)


def set_object_form(controls, object_id="motor_a"):
    controls.object_id_field.setText(object_id)
    controls.object_display_name_field.setText("Motor A")
    controls.reference_tag_id_field.setText("23")
    controls.reference_tag_family_field.setText("36h11")


def set_routine_form(controls, routine_id="thermal_scan"):
    controls.routine_id_field.setText(routine_id)
    controls.routine_display_name_field.setText("Thermal scan")
    controls.sensor_id_field.setCurrentIndex(
        controls.sensor_id_field.findData("mlx90640")
    )


def test_delete_buttons_are_outside_creation_dialog(controls):
    dialog_button_texts = {
        button.text()
        for button in controls.management_dialog.findChildren(QPushButton)
    }

    assert "Delete Object" not in dialog_button_texts
    assert "Delete Routine" not in dialog_button_texts
    assert controls.delete_object_button.isEnabled() is False
    assert controls.delete_routine_button.isEnabled() is False


def test_object_creation_updates_repository_and_dropdowns(controls):
    set_object_form(controls)

    assert controls.handle_create_object() is True
    assert controls.object_repository.exists("motor_a")
    assert controls.saved_object_dropdown.currentData() == "motor_a"
    assert (
        controls.routine_parent_object_dropdown.findData("motor_a")
        >= 0
    )


def test_routine_creation_uses_existing_object_dropdown(controls):
    save_object(controls)
    controls.refresh_saved_definitions()
    parent_index = controls.routine_parent_object_dropdown.findData(
        "motor_a"
    )
    controls.routine_parent_object_dropdown.setCurrentIndex(parent_index)
    controls.object_id_field.clear()
    set_routine_form(controls)

    assert controls.handle_create_routine() is True

    definition = controls.object_repository.load("motor_a")
    assert definition.get_routine("thermal_scan") is not None
    assert controls.saved_object_dropdown.currentData() == "motor_a"
    assert controls.saved_routine_dropdown.currentData() == (
        "thermal_scan"
    )


def test_delete_routine_removes_repository_and_dropdown_entry(controls):
    routine = InspectionRoutine(
        routine_id="thermal_scan",
        display_name="Thermal scan",
        sensor_id="mlx90640",
    )
    save_object(controls, routines=[routine])
    select_object(controls, "motor_a")
    select_routine(controls, "thermal_scan")

    assert controls.handle_delete_routine() is True

    definition = controls.object_repository.load("motor_a")
    assert definition.get_routine("thermal_scan") is None
    assert (
        controls.saved_routine_dropdown.findData("thermal_scan")
        == -1
    )
    assert controls.delete_routine_button.isEnabled() is False


def test_delete_object_removes_every_dropdown_entry(controls):
    save_object(controls)
    select_object(controls, "motor_a")

    assert controls.handle_delete_object() is True

    assert controls.object_repository.exists("motor_a") is False
    assert controls.saved_object_dropdown.findData("motor_a") == -1
    assert (
        controls.routine_parent_object_dropdown.findData("motor_a")
        == -1
    )
    assert controls.delete_object_button.isEnabled() is False


def test_deleted_object_id_can_be_created_again(controls):
    save_object(controls)
    select_object(controls, "motor_a")
    assert controls.handle_delete_object() is True

    set_object_form(controls, "motor_a")
    assert controls.handle_create_object() is True
    assert controls.object_repository.exists("motor_a")


def test_unloadable_object_remains_deletable(controls):
    object_dir = controls.object_repository.get_object_dir("broken")
    object_dir.mkdir(parents=True)
    controls.object_repository.get_object_path("broken").write_text(
        "not: [valid",
        encoding="utf-8",
    )

    controls.refresh_saved_definitions()
    index = controls.saved_object_dropdown.findData("broken")
    controls.saved_object_dropdown.setCurrentIndex(index)

    assert controls.delete_object_button.isEnabled() is True
    assert controls.handle_delete_object() is True
    assert object_dir.exists() is False


def test_cancelled_deletion_preserves_repository(controls):
    save_object(controls)
    select_object(controls, "motor_a")
    controls.ask_question = lambda title, message: False

    assert controls.handle_delete_object() is False
    assert controls.object_repository.exists("motor_a")


def test_missing_view_dataset_does_not_hide_available_preview(
    controls,
    monkeypatch,
):
    from types import SimpleNamespace

    from sensor_msgs.msg import CameraInfo, Image

    from fault_detector_spot.ui.controls import (
        inspection_controls as controls_module,
    )
    from fault_detector_spot.inspection.data.models import (
        PoseData,
        ReferenceView,
    )

    hand_view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/scan/set_1/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )
    left_view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="left_image_sensor",
        reference_dataset_path=(
            "reference_datasets/scan/set_1/slot2_left"
        ),
        view_id="slot2_left",
        camera_id="left",
        slot_index=1,
    )
    routine = InspectionRoutine(
        routine_id="scan",
        display_name="Scan",
        sensor_id="bmm150",
        reference_views=[hand_view, left_view],
    )
    save_object(controls, routines=[routine])

    rgb_image = Image()
    rgb_image.width = 2
    rgb_image.height = 2
    rgb_image.encoding = "rgb8"
    rgb_image.step = 6
    rgb_image.data = [0] * 12
    depth_image = Image()
    depth_image.width = 2
    depth_image.height = 2
    capture = SimpleNamespace(
        slot_index=0,
        camera_id="hand",
        reference_view=hand_view,
        rgb_image=rgb_image,
        depth_image=depth_image,
        rgb_camera_info=CameraInfo(),
        depth_camera_info=CameraInfo(),
    )

    def load_capture(
        object_id,
        routine_id,
        reference_tag_id,
        reference_view,
    ):
        if reference_view.camera_id == "left":
            raise FileNotFoundError("slot2_left/metadata.json")
        return capture

    monkeypatch.setattr(
        controls,
        "_load_reference_view_capture",
        load_capture,
    )
    monkeypatch.setattr(
        controls_module,
        "rgb_depth_selectable_region",
        lambda *args: None,
    )

    select_object(controls, "motor_a")
    select_routine(controls, "scan")

    assert controls.reference_view_widgets[0].has_image is True
    assert controls.reference_view_widgets[1].text() == (
        "Left unavailable"
    )
    assert "Hand" in controls.reference_view_status_label.text()
    assert "1 unavailable" in controls.reference_view_status_label.text()
