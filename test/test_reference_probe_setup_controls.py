"""Tests for transient probe setup commands in inspection controls."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from fault_detector_msgs.msg import BasicCommand, TagElement
from PyQt5.QtWidgets import QApplication, QLabel

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    OrientationModes,
)
from fault_detector_spot.behaviour_tree.ui_classes.inspection_controls import (
    InspectionControls,
)
from fault_detector_spot.inspection.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
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
    controls.probe_frame_field.setText("sensor_tip")
    controls._lookup_pose = lambda target, source: pose(x=1.30)

    current = controls._current_probe_pose_object()

    assert current.position.x == pytest.approx(0.30)
    assert current.position.y == pytest.approx(0.0)
    assert current.position.z == pytest.approx(0.0)


def test_setup_motion_publishes_existing_move_to_tag_command(
    application,
    tmp_path,
):
    ui = FakeUI(tmp_path)
    controls = InspectionControls(ui)
    configure_live_tag(controls, ui)
    controls._hand_to_probe_pose = lambda: PoseData.identity()
    controls._probe_setup = SimpleNamespace(
        safe_approach_pose_object=pose(x=0.20),
        aligned_preapproach_pose_object=pose(x=0.15),
        probe_pose_object=pose(x=0.03),
    )

    controls.handle_move_to_probe_pose()

    assert len(ui.complex_command_publisher.messages) == 1
    message = ui.complex_command_publisher.messages[0]
    assert message.command.command_id == "move_to_tag"
    assert message.offset.pose.position.x == pytest.approx(0.03)
    assert ui.status_label.text() == "Command sent: move to probe pose"
