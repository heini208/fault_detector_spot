"""Per-command map bypass survives transport without becoming a sticky UI mode."""

import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
from fault_detector_msgs.msg import OperationalIntent, TagElement

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin, CommandRequest, RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recorded_command, serialize_recorded_command,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message, command_request_to_message,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.ui.manipulation.controls import ManipulationControls


@pytest.mark.parametrize("bypass", [False, True])
def test_policy_survives_intent_command_transport_and_recording(bypass):
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_ARM_RELATIVE
    intent.offset.header.frame_id = "body"
    intent.ignore_environment_collisions = bypass
    semantic = operational_intent_to_command(intent)
    request = CommandRequest.create(
        command=semantic, client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )

    restored = command_request_from_message(command_request_to_message(request))
    recorded = deserialize_recorded_command(serialize_recorded_command(restored.command))

    assert restored.command == semantic
    assert recorded == semantic
    assert recorded.ignore_environment_collisions is bypass


def test_old_recording_uses_collision_checks():
    old = serialize_recorded_command(SemanticCommand(CommandID.MOVE_ARM_RELATIVE))
    old.pop("ignore_environment_collisions")

    assert deserialize_recorded_command(old).ignore_environment_collisions is False
    assert OperationalIntent().ignore_environment_collisions is False


@pytest.mark.parametrize("value", [1, "false", None])
def test_policy_rejects_non_booleans(value):
    with pytest.raises(TypeError, match="boolean"):
        SemanticCommand(CommandID.MOVE_ARM_RELATIVE, ignore_environment_collisions=value)
    record = serialize_recorded_command(SemanticCommand(CommandID.MOVE_ARM_RELATIVE))
    record["ignore_environment_collisions"] = value
    with pytest.raises(TypeError, match="boolean"):
        deserialize_recorded_command(record)


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    tag = TagElement()
    tag.id = 1
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    ui = SimpleNamespace(
        node=None, status_label=QLabel(), visible_tags={1: tag},
        update_frames_dropdown=lambda box: box.addItem("body"),
        update_tags_dropdown=lambda box: box.addItem("1"),
        execute_operation=Mock(return_value="request"),
        handle_simple_operation=Mock(),
    )
    controls = ManipulationControls(ui)
    controls.ask_question = Mock(return_value=True)
    yield controls
    controls.destroy()
    controls.posture_button.timer.stop()


@pytest.mark.parametrize("handler, args", [
    ("handle_tag_selection", ()),
    ("handle_move_and_wait", ()),
    ("handle_full_message", (OperationalIntent.INTENT_MOVE_ARM_RELATIVE,)),
    ("handle_orient_to_surface", ()),
    ("handle_orient_to_tag", ()),
    ("handle_move_close_to_surface", ()),
])
def test_basic_movement_consumes_override_once(controls, handler, args):
    controls.ignore_environment_collisions_checkbox.setChecked(True)

    getattr(controls, handler)(*args)
    first = controls.ui.execute_operation.call_args.args[0]
    assert first.ignore_environment_collisions is True
    assert not controls.ignore_environment_collisions_checkbox.isChecked()

    getattr(controls, handler)(*args)
    second = controls.ui.execute_operation.call_args.args[0]
    assert second.ignore_environment_collisions is False
    assert first.ignore_environment_collisions is True


def test_rejected_submission_still_consumes_override(controls):
    controls.ignore_environment_collisions_checkbox.setChecked(True)
    controls.ui.execute_operation.return_value = None

    controls.handle_orient_to_surface()

    assert controls.ui.execute_operation.call_args.args[0].ignore_environment_collisions
    assert not controls.ignore_environment_collisions_checkbox.isChecked()


def test_submission_exception_still_consumes_override(controls):
    controls.ignore_environment_collisions_checkbox.setChecked(True)
    controls.ui.execute_operation.side_effect = RuntimeError("admission rejected")

    with pytest.raises(RuntimeError, match="admission rejected"):
        controls.handle_orient_to_surface()

    assert not controls.ignore_environment_collisions_checkbox.isChecked()


def test_wait_and_base_actions_do_not_consume_override(controls):
    controls.ignore_environment_collisions_checkbox.setChecked(True)

    controls.handle_full_message(OperationalIntent.INTENT_WAIT)
    assert not controls.ui.execute_operation.call_args.args[0].ignore_environment_collisions
    controls.handle_full_message(OperationalIntent.INTENT_MOVE_BASE_RELATIVE)
    assert not controls.ui.execute_operation.call_args.args[0].ignore_environment_collisions

    assert controls.ignore_environment_collisions_checkbox.isChecked()


def test_cancelled_confirmation_does_not_consume_override(controls):
    controls.ignore_environment_collisions_checkbox.setChecked(True)
    controls.ask_question.return_value = False

    controls.handle_tag_selection()

    controls.ui.execute_operation.assert_not_called()
    assert controls.ignore_environment_collisions_checkbox.isChecked()


def test_gripper_button_does_not_consume_override(controls):
    controls.ignore_environment_collisions_checkbox.setChecked(True)
    robot_actions = controls.rows[-1].itemAt(0).widget()
    button = next(
        button for button in robot_actions.findChildren(QPushButton)
        if button.text() == "Gripper Toggle"
    )

    button.click()

    controls.ui.handle_simple_operation.assert_called_once_with(
        OperationalIntent.INTENT_TOGGLE_GRIPPER
    )
    assert controls.ignore_environment_collisions_checkbox.isChecked()
