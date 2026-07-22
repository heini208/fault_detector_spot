"""Tests for the compact inspection setup workspace."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
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
        self.visible_tags = {}

    def build_basic_command(self, command_id):
        command = BasicCommand()
        command.command_id = command_id
        return command


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


def test_workspace_uses_persistent_image_and_workflow_tabs(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))

    assert controls.inspection_workspace_splitter.orientation() == (
        Qt.Vertical
    )
    assert controls.inspection_workspace_splitter.count() == 2
    upper_policy = (
        controls.inspection_workspace_splitter.widget(0).sizePolicy()
    )
    lower_policy = (
        controls.inspection_workspace_splitter.widget(1).sizePolicy()
    )
    assert upper_policy.verticalStretch() == 2
    assert lower_policy.verticalStretch() == 3
    assert controls.reference_view_widget.minimumWidth() == 240
    assert controls.reference_view_widget.minimumHeight() == 180
    assert controls.reference_camera_two_placeholder.text() == (
        "No camera selected"
    )
    assert controls.reference_camera_three_placeholder.text() == (
        "No camera selected"
    )
    assert controls.workflow_tabs.count() == 3
    assert [
        controls.workflow_tabs.tabText(index)
        for index in range(controls.workflow_tabs.count())
    ] == ["Target", "Refine", "Save"]
    assert controls.geometry_details_section.content_frame.isHidden()
    assert not controls.save_probe_point_button.isEnabled()


def test_management_controls_live_in_non_modal_dialog(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))

    assert not controls.management_dialog.isModal()
    assert controls.object_id_field.window() is controls.management_dialog
    assert controls.reference_tag_id_field.window() is (
        controls.management_dialog
    )
    assert controls.routine_id_field.window() is controls.management_dialog
    assert controls.probe_frame_field.window() is controls.management_dialog


def test_transient_approval_statuses_update_all_tabs(
    application,
    tmp_path,
):
    controls = InspectionControls(FakeUI(tmp_path))
    controls._probe_setup = SimpleNamespace(
        safe_approach_approved=True,
        surface_alignment_approved=True,
        probe_pose_approved=False,
    )

    controls._update_probe_setup_status_widgets()

    assert controls.approach_step_status_label.text() == "Approved"
    assert controls.alignment_step_status_label.text() == "Approved"
    assert controls.probe_step_status_label.text() == (
        "Ready for refinement"
    )
    assert controls.save_approach_status_label.text() == "Approved"
    assert controls.save_alignment_status_label.text() == "Approved"
    assert controls.save_probe_status_label.text() == "Not approved"
