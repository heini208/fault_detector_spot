"""Offline coverage of profile selection through the UI and command wire."""

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import pytest
from PyQt5.QtWidgets import QApplication, QComboBox, QLineEdit
from fault_detector_msgs.msg import OperationalIntent, TagElement
from fault_detector_spot.ui.navigation.base_movement_controls import BaseMovementControls
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.recording.semantic_command_codec import (
    serialize_recorded_command, deserialize_recorded_command,
)
from fault_detector_spot.navigation.walking_profile import WalkingProfiles


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize("intent_id", [
    OperationalIntent.INTENT_MOVE_BASE_RELATIVE,
    OperationalIntent.INTENT_MOVE_BASE_TO_TAG,
])
def test_dropdown_selection_survives_wire_and_execution_translation(intent_id):
    controls = object.__new__(BaseMovementControls)
    tag = TagElement()
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    controls.ui = SimpleNamespace(visible_tags={7: tag})
    controls._make_walking_profile_row()
    controls.tag_dropdown = QComboBox()
    controls.tag_dropdown.addItem("7")
    controls.frames_dropdown = QComboBox()
    controls.frames_dropdown.addItem("body")
    controls.offset_fields = {key: QLineEdit("0.0") for key in ("X", "Y", "Yaw")}
    assert controls.walking_profile_dropdown.currentText() == "Normal"
    first = controls.build_move_base_intent(intent_id)
    controls.walking_profile_dropdown.setCurrentText("Precision")
    second = controls.build_move_base_intent(intent_id)
    assert first.walking_profile == "normal"
    assert second.walking_profile == "precision"
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: first.offset.header.stamp
    for intent in (first, second):
        semantic = operational_intent_to_command(intent)
        restored = semantic_command_from_message(semantic_command_to_message(semantic))
        recorded = deserialize_recorded_command(serialize_recorded_command(restored))
        execution = subscriber.fire_command_sequence(recorded)[0]
        assert execution.walking_profile == intent.walking_profile
        profiles = WalkingProfiles(relative_profile="precision", tag_profile="precision")
        selected = profiles.for_move(override=execution.walking_profile)
        assert selected == getattr(profiles, intent.walking_profile)
    legacy = serialize_recorded_command(semantic)
    legacy.pop("walking_profile")
    assert deserialize_recorded_command(legacy).walking_profile == ""


def test_invalid_wire_profile_rejected():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_BASE_RELATIVE
    intent.offset.header.frame_id = "body"
    intent.walking_profile = "typo"
    with pytest.raises(ValueError, match="Walking profile"):
        operational_intent_to_command(intent)
