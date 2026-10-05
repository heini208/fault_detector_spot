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
