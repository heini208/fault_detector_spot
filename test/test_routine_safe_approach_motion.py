"""Routine safe motion is independent of saved probe-point selection."""

from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import OperationalIntent, ApplicationCommandState

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.controllers.application_controller import ApplicationController
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.inspection.execution.saved_probe_motion import routine_safe_approach_command
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
from fault_detector_spot.shared.geometry.models import PoseData
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_application_controller import FakeCommandController
from test_probe_execution_target import sensor
from test_probe_point_guided_workflow import FakeUI, make_state, application
from test_routine_base_position_motion import _definition


def intent():
    value = OperationalIntent()
    value.intent = OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH
    value.object_id = "motor"
    value.routine_id = "scan"
    return value


def test_routine_motion_resolves_shared_pose_without_any_probe_points():
    definition = _definition()
    routine = definition.get_routine("scan")
    routine.safe_approach_position_tolerance_m = .045
    routine.safe_approach_pose_object = PoseData.identity()
    routine.safe_approach_pose_object.position.z = 0.3
    command = routine_safe_approach_command(
        intent(), Mock(load=Mock(return_value=definition)),
        Mock(require_motion_attachment=Mock(return_value=sensor())),
        ProbeSetupMotionCommandFactory(),
    )
    assert command.tag_position_tolerance_m == .045
    assert routine.probe_points == []
    assert command.command_id is CommandID.MOVE_SAFE_APPROACH
    assert command.inspection.object_id == "motor"
    assert command.inspection.routine_id == "scan"
    assert command.inspection.probe_point_id == ""
    assert command.offset.position.z == pytest.approx(0.3)
    assert command.motion_sensor_id == sensor().motion_sensor_id
    assert command.tag.id == 7
    assert command.tag.pose.frame_id == ""
    assert command.offset.frame_id == "filtered_fiducial_7"


def test_unconfigured_routine_pose_rejects_motion():
    with pytest.raises(ValueError, match="routine safe pre-approach"):
        routine_safe_approach_command(
            intent(), Mock(load=Mock(return_value=_definition())),
            Mock(), Mock(),
        )


@pytest.mark.parametrize("field", ["object_id", "routine_id"])
def test_adapter_requires_routine_selection_but_no_probe_point(field):
    value = intent()
    assert operational_intent_to_command(value).command_id is CommandID.MOVE_SAFE_APPROACH
    setattr(value, field, "")
    with pytest.raises(ValueError):
        operational_intent_to_command(value)


def test_application_controller_uses_routine_resolution():
    controller = ApplicationController(FakeCommandController())
    resolved = SemanticCommand(command_id=CommandID.MOVE_SAFE_APPROACH)
    coordinator = Mock()
    coordinator.uses_setup_coordinator.return_value = True
    coordinator.routine_safe_approach_command.return_value = resolved
    controller.attach_probe_setup(coordinator)
    value = intent()
    operation = controller.prepare_operation(value, "ui")
    coordinator.routine_safe_approach_command.assert_called_once_with(value)
    assert operation.request.command is resolved


def test_routine_row_move_works_without_point_selection_and_tracks_result(application):
    ui = FakeUI()
    ui.execute_operation = Mock(return_value="request")
    controls = FinalizingInspectionControls(ui)
    state = make_state(with_references=False)
    controls.apply_setup_state(state)
    button = controls.move_to_routine_arm_pose_button
    group = controls.set_routine_arm_pose_button.parentWidget()
    assert button.parentWidget() is group
    row = group.layout().itemAt(2).layout()
    assert row.itemAt(0).widget() is controls.set_routine_arm_pose_button
    assert row.itemAt(1).widget() is button
    assert controls.saved_probe_points_list.count() == 0
    assert OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH not in controls.saved_probe_action_buttons
    assert button.styleSheet() == controls.move_to_base_position_button.styleSheet()
    assert button.styleSheet() == ""
    assert button.isEnabled()
    assert controls.handle_move_to_routine_arm_pose()
    args, kwargs = ui.execute_operation.call_args
    assert args[0].intent == OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH
    assert args[0].probe_point_id == ""
    assert controls.handle_move_to_routine_arm_pose()
    second_context = ui.execute_operation.call_args.kwargs["context_id"]
    assert second_context != kwargs["context_id"]
    result = ApplicationCommandState()
    result.context_id = "unrelated"
    result.state = result.STATE_SUCCEEDED
    controls.handle_application_state(result)
    assert button.isEnabled()
    assert len(controls._routine_arm_pose_operation_contexts) == 2
    result.context_id = kwargs["context_id"]
    controls.handle_application_state(result)
    assert button.isEnabled()
    assert controls._routine_arm_pose_operation_contexts == {second_context}
    state.has_routine_safe_approach_pose = False
    controls.apply_setup_state(state)
    assert not button.isEnabled()
    assert not controls.handle_move_to_routine_arm_pose()


def test_routine_move_submission_rejection_allows_retry(application):
    ui = FakeUI()
    ui.execute_operation = Mock(return_value=None)
    controls = FinalizingInspectionControls(ui)
    controls.apply_setup_state(make_state())
    assert not controls.handle_move_to_routine_arm_pose()
    assert controls.move_to_routine_arm_pose_button.isEnabled()
    ui.execute_operation.return_value = "request"
    assert controls.handle_move_to_routine_arm_pose()
    context_id = ui.execute_operation.call_args.kwargs["context_id"]
    controls.handle_routine_arm_pose_rejected("Disconnected", context_id)
    assert controls.move_to_routine_arm_pose_button.isEnabled()
