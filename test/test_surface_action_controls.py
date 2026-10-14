"""Standalone sensor-tip surface actions use normal command admission and feedback."""

import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from builtin_interfaces.msg import Time
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin, CommandRequest, RecordingPolicy,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message, command_request_to_message,
)
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.ui.manipulation.controls import ManipulationControls
from test_sensor_attachment_controller import build_controller, sensor_definition


@pytest.fixture(scope="module")
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    ui = SimpleNamespace(
        node=None, status_label=QLabel(), visible_tags={},
        update_frames_dropdown=lambda box: box.addItem("body"),
        update_tags_dropdown=lambda box: None,
        execute_operation=Mock(return_value="tracked-operation"),
        handle_simple_operation=Mock(),
    )
    controls = ManipulationControls(ui)
    yield controls
    controls.destroy()
    controls.posture_button.timer.stop()


_ACTIONS = (
    ("test_surface_distance_button", OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE,
     CommandID.MEASURE_SENSOR_TIP_DISTANCE),
    ("orient_sensor_tip_to_surface_button", OperationalIntent.INTENT_ORIENT_SENSOR_TIP_TO_SURFACE,
     CommandID.ORIENT_SENSOR_TIP_TO_SURFACE),
)


def test_surface_group_exposes_both_sensor_actions_and_distinguishes_camera_action(controls):
    group = controls.rows[2].itemAt(0).widget()
    labels = {button.text() for button in group.findChildren(QPushButton)}
    assert group.toggle_button.text() == "Surface actions"
    assert not group.toggle_button.isChecked()
    assert group.content_frame.isHidden()
    group.toggle_button.click()
    assert not group.content_frame.isHidden()
    assert {"Orient camera to surface", "Orient sensor tip to surface", "Test surface distance",
            "Move sensor tip close to surface"} <= labels
    assert "lower-middle" in controls.orient_to_surface_button.toolTip()
    assert not hasattr(controls, "expected_tip_distance_input")
    assert controls.surface_distance_input.value() == 0.03


@pytest.mark.parametrize("button, intent_id, command_id", _ACTIONS)
def test_sensor_tip_actions_need_no_distance_estimate_and_keep_operation_feedback(
    controls, button, intent_id, command_id,
):
    controls.surface_distance_input.setValue(0.015)
    controls.ignore_environment_collisions_checkbox.setChecked(True)

    getattr(controls, button).click()

    intent, = controls.ui.execute_operation.call_args.args
    assert intent.intent == intent_id
    assert intent.aligned_preapproach_distance_m == 0.0
    assert not intent.ignore_environment_collisions
    assert controls.ignore_environment_collisions_checkbox.isChecked()
    assert operational_intent_to_command(intent).command_id is command_id
    assert intent.object_id == intent.routine_id == intent.probe_point_id == ""


@pytest.mark.parametrize("target", [0.015, 0.0])
def test_sensor_tip_approach_only_requests_target_gap_and_tolerance(controls, target):
    controls.surface_distance_input.setValue(target)
    controls.surface_tolerance_input.setValue(0.007)

    controls.move_close_to_surface_button.click()

    intent, = controls.ui.execute_operation.call_args.args
    command = operational_intent_to_command(intent)
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: Time()
    execution, = subscriber.fire_command_sequence(command)
    assert execution.command_id is CommandID.MOVE_CLOSE_TO_SURFACE
    assert intent.aligned_preapproach_distance_m == 0.0
    assert execution.target_surface_distance_m == pytest.approx(target)
    assert execution.surface_tolerance_m == pytest.approx(0.007)


@pytest.mark.parametrize("button, intent_id, command_id", _ACTIONS)
@pytest.mark.parametrize("expected", [0.0, 0.003, 0.25, 0.50])
def test_public_surface_actions_survive_admission_transport_and_execution_conversion(
    tmp_path, button, intent_id, command_id, expected,
):
    attachments, repository, _, commands = build_controller(tmp_path)
    repository.create(sensor_definition("lowered_tip"))
    pending = attachments.select_sensor("lowered_tip")
    attachments.confirm_sensor("lowered_tip", pending.attachment_revision)
    commands.add_request_preparer(attachments.prepare_request)
    accepted = []
    commands.add_accepted_listener(accepted.append)
    intent = OperationalIntent()
    intent.intent = intent_id
    intent.aligned_preapproach_distance_m = expected
    request = CommandRequest.create(
        command=operational_intent_to_command(intent), client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    commands.submit(request)
    admitted = accepted[0]
    restored = command_request_from_message(command_request_to_message(admitted))
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: Time()
    execution, = subscriber.fire_command_sequence(restored.command)
    assert execution.command_id is command_id
    if command_id is CommandID.ORIENT_SENSOR_TIP_TO_SURFACE:
        assert execution.motion_sensor_id == "lowered_tip"
        assert not hasattr(execution, "expected_distance_m")
    else:
        assert execution.expected_distance_m == (pytest.approx(expected) if expected else None)
    assert not execution.ignore_environment_collisions


@pytest.mark.parametrize("button, intent_id, command_id", _ACTIONS)
def test_public_surface_actions_reject_unconfirmed_attachment(tmp_path, button, intent_id, command_id):
    attachments, _, _, commands = build_controller(tmp_path)
    commands.add_request_preparer(attachments.prepare_request)
    intent = OperationalIntent()
    intent.intent = intent_id
    intent.aligned_preapproach_distance_m = 0.25
    request = CommandRequest.create(
        command=operational_intent_to_command(intent), client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    with pytest.raises(RuntimeError, match="confirmation is pending"):
        commands.submit(request)


@pytest.mark.parametrize("button, intent_id, command_id", _ACTIONS)
@pytest.mark.parametrize("expected", [-0.1, 0.001, 0.501, float("nan"), float("inf")])
def test_public_surface_actions_reject_invalid_observation_range(button, intent_id, command_id, expected):
    intent = OperationalIntent()
    intent.intent = intent_id
    intent.aligned_preapproach_distance_m = expected
    with pytest.raises(ValueError, match="Expected sensor tip distance"):
        operational_intent_to_command(intent)


def test_renamed_camera_action_retains_existing_command(controls):
    controls.orient_to_surface_button.click()
    intent = controls.ui.execute_operation.call_args.args[0]
    assert intent.intent == OperationalIntent.INTENT_ORIENT_TO_SURFACE
    assert operational_intent_to_command(intent).command_id is CommandID.ORIENT_TO_SURFACE
