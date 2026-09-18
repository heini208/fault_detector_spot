"""Offline coverage for browsing and moving saved probe points."""
import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication
from fault_detector_msgs.msg import OperationalIntent, ApplicationCommandState
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.inspection.execution.saved_probe_motion import saved_probe_command
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from test_probe_point_guided_workflow import FakeUI, make_state
from test_probe_execution_target import inspection_object, sensor
from fault_detector_msgs.msg import TagElement


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls():
    ui = FakeUI()
    ui.execute_operation = Mock(return_value="local-request")
    controls = FinalizingInspectionControls(ui)
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two"]
    controls.apply_setup_state(state)
    return controls


def test_list_updates_and_selection_does_not_cross_routines(controls):
    listing = controls.saved_probe_points_list
    assert [listing.item(i).text() for i in range(listing.count())] == ["one", "two"]
    listing.setCurrentRow(1)
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two", "three"]
    controls.apply_setup_state(state)
    assert listing.currentItem().text() == "two"
    state.selected_routine_id = "other"
    controls.apply_setup_state(state)
    assert listing.currentItem() is None
    assert not any(b.isEnabled() for b in controls.saved_probe_action_buttons.values())


@pytest.mark.parametrize("operation", [24, 25, 26])
def test_click_sends_selected_ids_and_disables_duplicates(controls, operation):
    controls.saved_probe_points_list.setCurrentRow(1)
    assert controls.handle_saved_probe_motion(operation)
    args, kwargs = controls.ui.execute_operation.call_args
    intent = args[0]
    assert (intent.object_id, intent.routine_id, intent.probe_point_id) == ("motor", "scan", "two")
    assert intent.intent == operation
    assert not controls.handle_saved_probe_motion(operation)
    state = ApplicationCommandState()
    state.context_id = "unrelated"
    state.state = state.STATE_SUCCEEDED
    controls.handle_application_state(state)
    assert controls._saved_probe_operation_context
    state.context_id = kwargs["context_id"]
    state.state = state.STATE_FAILED
    state.detail = "No live tag"
    controls.handle_application_state(state)
    assert "No live tag" in controls.saved_probe_motion_status.text()
    assert controls.saved_probe_action_buttons[operation].isEnabled()


def test_transport_rejection_allows_retry(controls):
    controls.saved_probe_points_list.setCurrentRow(0)
    controls.handle_saved_probe_motion(24)
    controls.handle_saved_probe_rejected("Disconnected")
    assert controls.handle_saved_probe_motion(24)


@pytest.mark.parametrize("operation, field", [
    (24, "safe_approach_pose_object"), (25, "aligned_preapproach_pose_object"),
])
def test_saved_pose_uses_repository_geometry_and_live_tag(operation, field):
    definition = inspection_object()
    tag = TagElement()
    tag.id = 2
    tag.pose.header.frame_id = "body"
    tag.pose.pose.orientation.w = 1.0
    tag.pose.pose.position.y = 2.0
    source = Mock()
    source.reference_tag.return_value = tag
    intent = saved_intent(operation)
    command = saved_probe_command(intent, Mock(load=Mock(return_value=definition)), source,
                                  Mock(require_motion_attachment=Mock(return_value=sensor())),
                                  ProbeSetupMotionCommandFactory())
    pose = getattr(definition.routines[0].probe_points[0], field)
    assert command.command_id == CommandID.MOVE_ARM_TO_TAG
    assert command.offset.position.x == pose.position.x
    assert command.offset.orientation.w == pose.orientation.w
    assert command.inspection.probe_point_id == "point_1"
    source.reference_tag.assert_called_once_with(2)
    assert source.validate_aligned_probe_distance.call_count == (operation == 25)


def saved_intent(operation):
    intent = OperationalIntent()
    intent.intent = operation
    intent.object_id = "motor_a"
    intent.routine_id = "scan"
    intent.probe_point_id = "point_1"
    return intent


def test_close_surface_uses_saved_distance_not_ui_values():
    intent = saved_intent(26)
    intent.target_surface_distance_m = 9.0
    command = saved_probe_command(intent, Mock(load=Mock(return_value=inspection_object())), None,
                                  Mock(require_motion_attachment=Mock(return_value=sensor())), None)
    assert command.command_id == CommandID.MOVE_CLOSE_TO_SURFACE
    assert command.target_surface_distance_m == 0.03
    assert command.aligned_preapproach_distance_m == 0.10


@pytest.mark.parametrize("field", ["routine_id", "probe_point_id"])
def test_missing_saved_selection_rejected(field):
    intent = saved_intent(24)
    setattr(intent, field, "missing")
    with pytest.raises(ValueError, match="does not exist"):
        saved_probe_command(intent, Mock(load=Mock(return_value=inspection_object())), None, None, None)


@pytest.mark.parametrize("operation", [24, 25, 26])
def test_adapter_requires_saved_selection(operation):
    intent = saved_intent(operation)
    operational_intent_to_command(intent)
    intent.probe_point_id = ""
    with pytest.raises(ValueError):
        operational_intent_to_command(intent)


def test_distance_defaults_per_point_and_override_survives_refresh(controls):
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two"]
    state.probe_point_target_surface_distances_m = [0.03, 0.045]
    controls.apply_setup_state(state)
    controls.saved_probe_points_list.setCurrentRow(0)
    assert controls.saved_probe_distance.value() == 0.03
    controls.saved_probe_distance.setValue(0.02)
    controls.apply_setup_state(state)
    assert controls.saved_probe_distance.value() == 0.02
    controls.saved_probe_points_list.setCurrentRow(1)
    assert controls.saved_probe_distance.value() == 0.045
    controls.saved_probe_distance.setValue(0.025)
    assert controls.handle_saved_probe_motion(26)
    intent = controls.ui.execute_operation.call_args.args[0]
    assert intent.override_target_surface_distance
    assert intent.target_surface_distance_m == 0.025
    assert list(state.probe_point_target_surface_distances_m) == [0.03, 0.045]


@pytest.mark.parametrize("target", [0.0, 0.02, 0.05])
def test_close_distance_override_does_not_modify_saved_point(target):
    definition = inspection_object()
    intent = saved_intent(26)
    intent.override_target_surface_distance = True
    intent.target_surface_distance_m = target
    command = saved_probe_command(intent, Mock(load=Mock(return_value=definition)), None,
                                  Mock(require_motion_attachment=Mock(return_value=sensor())), None)
    assert command.target_surface_distance_m == target
    assert definition.routines[0].probe_points[0].target_surface_distance_m == 0.03


@pytest.mark.parametrize("target", [-0.01, float("nan"), float("inf"), 0.06])
def test_invalid_close_distance_override_rejected(target):
    intent = saved_intent(26)
    intent.override_target_surface_distance = True
    intent.target_surface_distance_m = target
    with pytest.raises(ValueError):
        saved_probe_command(intent, Mock(load=Mock(return_value=inspection_object())), None,
                            Mock(require_motion_attachment=Mock(return_value=sensor())), None)


def test_application_controller_resolves_saved_motion_before_submission():
    from test_application_controller import FakeCommandController
    from fault_detector_spot.application.controllers.application_controller import ApplicationController
    from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
    controller = ApplicationController(FakeCommandController())
    resolved = SemanticCommand(command_id=CommandID.MOVE_CLOSE_TO_SURFACE, target_surface_distance_m=0.02)
    coordinator = Mock(saved_probe_command=Mock(return_value=resolved))
    controller.attach_probe_setup(coordinator)
    intent = saved_intent(26)
    operation = controller.prepare_operation(intent, "ui")
    coordinator.saved_probe_command.assert_called_once_with(intent)
    assert operation.request.command is resolved
    assert operation.intent == 26


def test_newly_saved_point_is_not_reported_as_duplicate_during_finalization(
    controls,
):
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two", "three"]
    state.refinement_active = True
    controls._probe_setup_state = state
    controls._probe_finalization_point_id = "three"
    controls._probe_finalization_scope = ("motor", "scan")
    controls.probe_point_id_field.setText("three")
    controls.probe_point_display_name_field.setText("Three")

    controls._update_save_probe_point_state()

    assert controls.save_probe_point_status_label.text() == (
        "Probe point saved. Mandatory retraction in progress."
    )
    assert not controls.approve_and_retract_button.isEnabled()


def test_completed_finalization_selects_new_point_and_enables_motion(controls):
    state = make_state(with_references=False)
    state.probe_point_ids = ["one", "two", "three"]
    state.probe_point_target_surface_distances_m = [0.03, 0.04, 0.05]
    controls._probe_setup_state = state
    controls._update_saved_probe_points(state)
    controls._probe_finalization_point_id = "three"
    controls._probe_finalization_scope = ("motor", "scan")

    controls._finish_refinement_workflow_close()
    controls._update_saved_probe_points(state)

    assert controls.saved_probe_points_list.currentItem().text() == "three"
    assert controls.saved_probe_distance.value() == 0.05
    assert all(
        button.isEnabled()
        for button in controls.saved_probe_action_buttons.values()
    )
    assert controls.saved_probe_motion_status.text() == "three: ready."


def test_close_is_blocked_while_save_finalization_is_running(controls):
    controls._probe_finalization_point_id = "three"
    controls._refinement_presentation = SimpleNamespace(
        recovery_required=False,
        pending_motion=None,
    )

    assert controls.request_close_refinement_workflow() is False
    assert "mandatory retraction" in (
        controls.refinement_recovery_status_label.text()
    )
