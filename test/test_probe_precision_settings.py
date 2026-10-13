"""Precision settings survive authoring and reach each movement command."""

from copy import deepcopy
from dataclasses import asdict
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import OperationalIntent, ProbeSetupIntent, ProbeSetupState

from fault_detector_spot.inspection.execution.saved_probe_motion import saved_probe_command
from fault_detector_spot.inspection.model.models import PreApproachPathPoint, ProbePoint
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind, ProbeMotionRequest, ProbeSetupMotionCommandFactory,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
from fault_detector_spot.shared.geometry.movement_frames import OrientationModes
from fault_detector_spot.ui.inspection.pre_approach_path_dialog import PreApproachPathDialog
from fault_detector_spot.ui.manipulation.controls import ManipulationControls
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from test_command_request_correlation import FakeClock
from test_custom_probe_workflow import begin_custom, latest, move
from test_probe_execution_target import inspection_object, sensor
from test_probe_point_guided_workflow import FakeUI, application
from test_probe_safe_approach_navigation import safe_state
from test_probe_setup_coordinator import pose
from test_saved_probe_controls import saved_intent
from test_tag_selection_dropdowns import _Parent


def test_legacy_saved_points_default_to_normal_and_flags_round_trip():
    point = inspection_object().routines[0].probe_points[0]
    legacy = point.to_dict()
    del legacy["aligned_precision_mode"], legacy["final_precision_mode"]
    restored = ProbePoint.from_dict(legacy)
    assert restored.aligned_precision_mode is restored.final_precision_mode is False
    point.aligned_precision_mode = True
    point.final_precision_mode = True
    point.pre_approach_path = [PreApproachPathPoint("precision", pose(.8), .02, .5, True)]
    assert ProbePoint.from_dict(point.to_dict()) == point
    legacy_waypoint = point.pre_approach_path[0].to_dict()
    del legacy_waypoint["precision_mode"]
    assert PreApproachPathPoint.from_dict(legacy_waypoint).precision_mode is False


def test_path_settings_update_does_not_recapture_geometry_and_rejects_invalid(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    probe.motion_state_source.pose = pose(.8)
    state = probe.add_pathing_point(state.context, "first", .01, .7)
    original = deepcopy(state.pre_approach_path[0].pose_object)
    probe.motion_state_source.pose = pose(.2)
    state = probe.set_pathing_point_settings(state.context, 0, .023, .4, True)
    point = state.pre_approach_path[0]
    assert (point.position_tolerance_m, point.arm_speed_scale, point.precision_mode) == (.023, .4, True)
    assert point.pose_object == original
    with pytest.raises(ValueError, match="precision"):
        probe.set_pathing_point_settings(state.context, 0, .02, .5, "false")
    assert latest(probe, state).pre_approach_path[0] == point


def test_custom_setup_keeps_modes_per_stage_and_waypoint_through_save(tmp_path):
    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    state = probe.add_pathing_point(state.context, "pre", .025, .7, precision_mode=True)
    probe.motion_state_source.pose = pose(.8)
    state = probe.approve_aligned_pose(state.context, .02, .8, True)
    state = probe.add_pathing_point(state.context, "final", .015, .15, "probe", True)
    probe.motion_state_source.pose = pose(.9)
    state = probe.approve_probe_pose(state.context, .005, .2, False)
    message = ProbeSetupStateAdapter(FakeClock()).message(state, 0, ProbeSetupState.STATE_READY, "Ready")
    assert message.aligned_precision_mode is True and message.final_precision_mode is False
    assert list(message.pathing_point_precision_modes) == [True]
    assert list(message.final_pathing_point_precision_modes) == [True]
    # A new choice applies to this test movement before another pose save.
    state, command = move(probe, commands, state, ProbeMotionKind.MOVE_PRE_APPROACH_PATH,
                          "probe", achieved=state.setup.probe_pose_object, precision_mode=True)
    assert command.precision_mode is True
    assert command.pre_approach_precision_modes == (True,)
    request_id = "9b361339-3d85-4a2e-a5c3-88cf9a6d4872"
    state = probe.begin_finalization(state.context, request_id, True)
    state = probe.approve_probe_geometry_for_finalization(state.context, request_id)
    state = probe.save_probe_point_for_finalization(state.context, request_id, "custom", "Custom", .01, .1, 1.)
    point = probe.object_repository.load("motor").get_routine("magnetic_scan").get_probe_point("custom")
    assert point.aligned_precision_mode is True and point.final_precision_mode is False
    assert point.pre_approach_path[0].precision_mode is True
    assert point.final_probe_path[0].precision_mode is True
    retreat = probe.prepare_motion(state.context, ProbeMotionRequest(
        ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH, retract_path=True),
        finalization_request_id=request_id).request.command
    assert retreat.precision_mode is True
    assert retreat.pre_approach_precision_modes == (True,)


@pytest.mark.parametrize("operation, custom", [
    (OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH, False),
    (OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH, True),
    (OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE, False),
])
def test_saved_commands_keep_precision_and_nominal_close_return_target(operation, custom):
    definition = inspection_object()
    routine = definition.routines[0]
    point = routine.probe_points[0]
    point.aligned_precision_mode = True
    point.final_precision_mode = True
    if custom:
        point.fully_custom = True
        point.target_surface_distance_m = point.aligned_preapproach_distance_m = 0.
        point.final_probe_pose_object = pose(.9)
    points = [PreApproachPathPoint("first", pose(.7), .02, .5, True),
              PreApproachPathPoint("second", pose(.8), .01, .2, False)]
    point.final_probe_path = points if custom else []
    point.pre_approach_path = [] if custom else points
    command = saved_probe_command(saved_intent(operation), Mock(load=Mock(return_value=definition)),
                                  Mock(), Mock(require_motion_attachment=Mock(return_value=sensor())),
                                  ProbeSetupMotionCommandFactory())
    assert command.precision_mode is True
    assert command.tag.id == routine.reference_tag.tag_id
    if operation == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE:
        assert asdict(command.offset.position) == asdict(point.aligned_preapproach_pose_object.position)
        assert command.orientation_mode == OrientationModes.CUSTOM_ORIENTATION.value
        assert command.target_surface_distance_m == point.target_surface_distance_m
        assert command.inspection.probe_point_id == point.probe_point_id
    else:
        assert command.pre_approach_precision_modes == (True, False)


def test_main_manipulator_mode_and_tolerance_reach_tag_and_relative_intents(application, monkeypatch):
    controls = object.__new__(ManipulationControls)
    controls.ui = _Parent([2])
    controls._make_tag_input_row()
    controls._make_precision_mode_row()
    controls._make_reset_fields_and_move_relative_row()
    monkeypatch.setattr(controls, "add_tag_element_to_intent", lambda intent, *_: intent)
    monkeypatch.setattr(controls, "add_offset_to_intent", lambda intent: intent)
    calls = []
    monkeypatch.setattr(controls, "_execute_basic_movement", calls.append)
    assert controls.precision_mode_combo.currentText() == "Normal"
    assert controls.add_tag_info_to_intent(OperationalIntent()).precision_mode is False
    controls.precision_mode_combo.setCurrentText("Precision")
    controls.tag_tolerance_input.setValue(.023)
    intent = controls.add_tag_info_to_intent(OperationalIntent())
    assert (intent.precision_mode, intent.tag_position_tolerance_m) == (True, .023)
    controls.relative_tolerance_input.setValue(.017)
    controls.handle_full_message(OperationalIntent.INTENT_MOVE_ARM_RELATIVE)
    assert (calls[-1].precision_mode, calls[-1].tag_position_tolerance_m) == (True, .017)


def test_waypoint_ui_loads_and_saves_all_selected_settings(application):
    controls = Mock()
    controls._path_stage.return_value = "alignment"
    controls._submit_probe_setup.return_value = None
    dialog = PreApproachPathDialog(controls, None)
    state = ProbeSetupState()
    state.refinement_active = True
    state.pathing_point_names = ["first", "second"]
    state.pathing_point_tolerances_m = [.021, .015]
    state.pathing_point_speed_scales = [.5, .2]
    state.pathing_point_precision_modes = [True, False]
    dialog.refresh(state, True)
    assert dialog.precision_checkbox.isChecked()
    assert dialog.tolerance_field.value() == .021
    dialog.points.setCurrentRow(1)
    assert not dialog.precision_checkbox.isChecked()
    assert dialog.tolerance_field.value() == .015
    dialog.precision_checkbox.setChecked(True)
    dialog.tolerance_field.setValue(.025)
    dialog.speed_field.setValue(30)
    dialog.save_speed_button.click()
    intent = controls._submit_probe_setup.call_args.args[0]
    assert intent.operation == ProbeSetupIntent.OPERATION_SET_PATHING_POINT_SETTINGS
    assert (intent.pathing_point_index, intent.precision_mode) == (1, True)
    assert (intent.position_tolerance_m, intent.arm_speed_scale) == (.025, .3)
    dialog.close()


def test_candidate_ui_separates_aligned_and_final_modes_and_saves_choice(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.fully_custom = True
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.aligned_precision_mode = True
    state.final_precision_mode = False
    controls.apply_setup_state(state)
    controls.refinement_dialog.show_stage(RefinementStage.ALIGNMENT)
    assert controls.probe_precision_checkbox.isChecked()
    controls.refinement_dialog.show_stage(RefinementStage.PROBE)
    assert not controls.probe_precision_checkbox.isChecked()
    controls.probe_precision_checkbox.setChecked(True)
    controls.handle_use_current_alignment()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_APPROVE_CUSTOM_PROBE
    assert ui.requests[-1].precision_mode is True
    controls.refinement_dialog.hide()


@pytest.mark.parametrize("reached", [True, False])
def test_setup_close_carries_only_current_reached_alignment_anchor(application, reached):
    ui = FakeUI()
    ui.execute_operation = Mock(return_value="operation")
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.context_id = "context"
    state.selected_reference_tag_id = 7
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED if reached else state.MOTION_NOT_TESTED
    state.aligned_precision_mode = True
    state.aligned_position_tolerance_m = .023
    controls.apply_setup_state(state)
    controls.refinement_dialog.show_stage(RefinementStage.ALIGNMENT)
    assert controls.handle_test_surface_distance()
    intent = ui.execute_operation.call_args.args[0]
    if reached:
        assert intent.offset.header.frame_id == "filtered_fiducial_7"
        assert intent.offset.pose == state.aligned_preapproach_candidate_pose_object
        assert intent.orientation_mode == OrientationModes.CUSTOM_ORIENTATION.value
        assert (intent.precision_mode, intent.tag_position_tolerance_m) == (True, .023)
    else:
        assert intent.offset.header.frame_id == ""
    controls.refinement_dialog.hide()
