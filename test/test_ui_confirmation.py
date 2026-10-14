"""A negative dialog reply must not authorize recording or movement requests."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtWidgets import QApplication, QMessageBox, QWidget
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.ui.shared.control_helper import UIControlHelper
from fault_detector_spot.ui.recording.controls import RecordingControls
from fault_detector_spot.ui.manipulation.controls import ManipulationControls
from fault_detector_spot.ui.navigation.base_movement_controls import BaseMovementControls


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize("reply", [QMessageBox.Yes, QMessageBox.No, QMessageBox.NoButton])
@pytest.mark.parametrize("operation", ["overwrite", "delete"])
def test_recording_confirmation_controls_submission(application, monkeypatch, reply, operation):
    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_state(False)
    controls.record_name_field.setText("existing")
    controls.recordings_dropdown.clear()
    controls.recordings_dropdown.addItem("existing")
    question = Mock(return_value=reply)
    monkeypatch.setattr(QMessageBox, "question", question)
    if operation == "overwrite":
        controls.toggle_recording()
    else:
        controls.delete_selected_recording()
    assert question.call_args.args[-1] == QMessageBox.No
    publish = controls.record_control_pub.publish
    if reply == QMessageBox.Yes:
        publish.assert_called_once()
        message = publish.call_args.args[0]
        assert message.mode == ("start" if operation == "overwrite" else "delete")
        assert message.name == "existing"
    else:
        publish.assert_not_called()
        assert controls.record_button.text() == "Start Recording"
        assert controls.record_button.styleSheet() == ""
    parent.close()


@pytest.mark.parametrize("reply", [QMessageBox.Yes, QMessageBox.No])
@pytest.mark.parametrize("handler", [
    ManipulationControls.handle_move_and_wait,
    ManipulationControls.handle_tag_selection,
    BaseMovementControls.handle_move_base_relative,
    BaseMovementControls.handle_move_to_tag,
])
def test_movement_callers_use_boolean_confirmation(monkeypatch, reply, handler):
    intent = OperationalIntent()
    intent.tag.id = 7
    execute = Mock()
    controls = SimpleNamespace(
        ui=SimpleNamespace(execute_operation=execute, visible_tags={7: object()}),
        status_label=Mock(),
        duration_input=SimpleNamespace(value=lambda: 1.0),
        tag_dropdown=SimpleNamespace(currentText=lambda: "7"),
        build_move_to_tag_intent=lambda: intent,
        build_move_base_intent=lambda _kind: intent,
        reset_body_height_on_move=False,
        ignore_environment_collisions_checkbox=Mock(isChecked=Mock(return_value=True)),
    )
    controls._execute_basic_movement = lambda intent: (
        ManipulationControls._execute_basic_movement(controls, intent)
    )
    controls._submit_base_move = lambda intent: (
        BaseMovementControls._submit_base_move(controls, intent)
    )
    controls.ask_question = lambda title, message: UIControlHelper.ask_question(
        controls, title, message,
    )
    monkeypatch.setattr(QMessageBox, "question", lambda *_args: reply)
    handler(controls)
    if reply == QMessageBox.Yes:
        execute.assert_called_once_with(intent)
    else:
        execute.assert_not_called()
    if reply == QMessageBox.Yes and handler in (
        ManipulationControls.handle_move_and_wait,
        ManipulationControls.handle_tag_selection,
    ):
        assert intent.ignore_environment_collisions
        controls.ignore_environment_collisions_checkbox.setChecked.assert_called_once_with(False)
    else:
        controls.ignore_environment_collisions_checkbox.setChecked.assert_not_called()


def test_recording_button_waits_for_backend_state(application):
    from std_msgs.msg import Bool

    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    callbacks = {
        call.args[1]: call.args[2]
        for call in parent.node.create_subscription.call_args_list
    }
    acknowledge = callbacks["fault_detector/recording_state"]
    assert not controls.record_button.isEnabled()
    controls.toggle_recording()
    controls.record_control_pub.publish.assert_not_called()

    acknowledge(Bool(data=False))
    application.processEvents()
    controls.record_name_field.setText("run")
    controls.toggle_recording()
    assert controls.record_control_pub.publish.call_args.args[0].mode == "start"
    assert controls.record_button.text() == "Start Recording"
    assert controls.record_button.styleSheet() == ""

    # Rejected start leaves the backend and button idle.
    acknowledge(Bool(data=False))
    application.processEvents()
    assert controls.record_button.text() == "Start Recording"
    # A successful start, including one from another client, updates the button.
    acknowledge(Bool(data=True))
    application.processEvents()
    assert controls.record_button.text() == "Stop Recording"
    assert not controls.record_name_field.isEnabled()
    controls.toggle_recording()
    assert controls.record_control_pub.publish.call_args.args[0].mode == "stop"
    assert controls.record_button.text() == "Stop Recording"
    acknowledge(Bool(data=True))  # Saving failed: recording remains active.
    application.processEvents()
    assert controls.record_button.text() == "Stop Recording"
    acknowledge(Bool(data=False))
    application.processEvents()
    assert controls.record_button.text() == "Start Recording"
    assert controls.record_name_field.isEnabled()
    parent.close()


@pytest.mark.parametrize("paused_first", [True, False])
def test_recording_pause_uses_confirmed_state(application, paused_first):
    from std_msgs.msg import Bool

    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    callbacks = {
        call.args[1]: call.args[2]
        for call in parent.node.create_subscription.call_args_list
    }
    recording = callbacks["fault_detector/recording_state"]
    paused = callbacks["fault_detector/recording_paused"]
    assert not controls.pause_recording_button.isEnabled()
    # A newly attached UI also reflects a session paused by another client.
    for callback in ([paused, recording] if paused_first else [recording, paused]):
        callback(Bool(data=True))
        application.processEvents()
    assert controls.pause_recording_button.text() == "Recording paused"
    assert controls.pause_recording_button.isChecked()
    assert "red" in controls.pause_recording_button.styleSheet()
    controls.record_control_pub.publish.assert_not_called()

    controls.pause_recording_button.click()
    assert controls.record_control_pub.publish.call_args.args[0].mode == "resume"
    assert controls.pause_recording_button.text() == "Recording paused"
    paused(Bool(data=False))
    application.processEvents()
    assert controls.pause_recording_button.text() == "Don't record"
    assert not controls.pause_recording_button.isChecked()
    controls.pause_recording_button.click()
    assert controls.record_control_pub.publish.call_args.args[0].mode == "pause"
    assert controls.pause_recording_button.text() == "Don't record"
    paused(Bool(data=True))
    application.processEvents()
    assert controls.pause_recording_button.text() == "Recording paused"
    recording(Bool(data=False))
    paused(Bool(data=False))
    application.processEvents()
    assert not controls.pause_recording_button.isEnabled()
    assert controls.pause_recording_button.text() == "Don't record"
    assert controls.record_control_pub.publish.call_count == 2
    parent.close()


def test_recording_waits_for_pending_outcomes_in_ui(application):
    from std_msgs.msg import Bool

    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_state(True)
    callbacks = {
        call.args[1]: call.args[2]
        for call in parent.node.create_subscription.call_args_list
    }
    callbacks["fault_detector/recording_finishing"](Bool(data=True))
    application.processEvents()
    assert controls.record_button.text() == "Finishing recording…"
    assert not controls.record_button.isEnabled()
    assert not controls.recording_options.isEnabled()
    controls.toggle_recording()
    controls.record_control_pub.publish.assert_not_called()
    # If saving fails after outcomes arrive, the operator can retry saving.
    callbacks["fault_detector/recording_finishing"](Bool(data=False))
    application.processEvents()
    assert controls.record_button.isEnabled()
    controls.toggle_recording()
    assert controls.record_control_pub.publish.call_args.args[0].mode == "stop"
    controls.update_recording_state(False)
    assert controls.record_button.text() == "Start Recording"
    parent.close()


def test_recording_feedback_displays_backend_result_even_when_controls_disabled(application):
    import json
    from std_msgs.msg import String

    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    callbacks = {
        call.args[1]: call.args[2]
        for call in parent.node.create_subscription.call_args_list
    }
    assert controls.recording_feedback_label.isEnabled()
    controls.skip_button.setChecked(True)
    controls.retry_button.setChecked(True)
    for outcome, expected in (
        ("pending", "Waiting for result: Stand up"),
        ("added", "Added: Stand up"),
        ("paused", "Not recorded · paused: Stand up"),
    ):
        callbacks["fault_detector/recording_feedback"](String(data=json.dumps({
            "command_id": "stand_up", "outcome": outcome,
        })))
        application.processEvents()
        assert controls.recording_feedback_label.text() == expected
        assert controls.recording_feedback_label.toolTip() == expected
    controls.record_control_pub.publish.assert_not_called()
    parent.close()


def test_retry_button_waits_for_backend_and_can_clear_setting(application):
    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_state(True)
    assert not controls.retry_button.isEnabled()
    controls.update_recording_retries(0)
    controls.retry_count.setValue(3)
    controls.retry_button.click()
    message = controls.record_control_pub.publish.call_args.args[0]
    assert message.mode == "set_retries" and message.retries == 3
    assert not controls.retry_button.isChecked()
    controls.update_recording_retries(3)
    assert controls.retry_button.isChecked()
    assert not controls.retry_count.isEnabled()
    controls.retry_button.click()
    assert controls.record_control_pub.publish.call_args.args[0].retries == 0
    assert controls.retry_button.isChecked()
    controls.update_recording_retries(0)
    assert not controls.retry_button.isChecked()
    assert controls.retry_count.isEnabled()
    parent.close()


@pytest.mark.parametrize('choice', ['skip_start', 'skip_continue', None])
def test_adjacent_skip_blocks_prompt_and_wait_for_confirmation(application, monkeypatch, choice):
    import json
    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_state(True)
    controls.update_recording_skip(json.dumps({
        'active_block': 0, 'adjacent_block': 1, 'next_block': 2, 'recording_name': 'run',
    }))
    prompt = Mock(return_value=choice)
    monkeypatch.setattr(controls, '_choose_adjacent_skip_block', prompt)
    controls.skip_button.click()
    prompt.assert_called_once_with(1, 2)
    assert not controls.skip_button.isChecked()
    if choice is None:
        controls.record_control_pub.publish.assert_not_called()
    else:
        request = controls.record_control_pub.publish.call_args.args[0]
        assert request.mode == choice
        assert request.skip_block == 1
        assert request.name == 'run'
    controls.update_recording_skip(json.dumps({
        'active_block': 2, 'adjacent_block': 0, 'next_block': 3, 'recording_name': 'run',
    }))
    assert controls.skip_button.text() == 'Stop skip block 2'
    assert 'red' in controls.skip_button.styleSheet()
    controls.skip_button.click()
    assert controls.skip_button.isChecked()
    assert controls.record_control_pub.publish.call_args.args[0].mode == 'skip_stop'
    parent.close()


def test_recording_feedback_shows_block_and_retries(application):
    import json
    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_feedback(json.dumps({
        'outcome': 'added', 'command_id': 'move_to_waypoint', 'skip_block': 1, 'playback_retries': 2,
    }))
    assert controls.recording_feedback_label.text() == 'Added: Move to waypoint · skip block 1 · 2 retries'
    parent.close()


def test_skip_button_waits_for_pending_outside_command(application):
    import json
    parent = QWidget()
    parent.node = Mock()
    controls = RecordingControls(parent)
    controls.update_recording_state(True)
    controls.update_recording_skip(json.dumps({
        'active_block': 0, 'adjacent_block': 0, 'next_block': 2,
        'recording_name': 'run', 'boundary_pending': True,
    }))
    assert not controls.skip_button.isEnabled()
    controls.toggle_skip_block()
    controls.record_control_pub.publish.assert_not_called()
    parent.close()
