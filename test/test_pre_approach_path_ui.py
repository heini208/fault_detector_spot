"""Path popup state and intents, independent of a running robot."""

from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupMotionIntent, ProbeSetupState
from test_probe_point_guided_workflow import application, FakeUI
from test_probe_safe_approach_navigation import safe_state
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage


def test_popup_captures_names_reorders_and_emits_selected_move(application):
    ui = FakeUI()
    motions = []
    ui.probe_setup_client.execute_motion = lambda intent: motions.append(intent) or "motion"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.safe_approach_motion_state = ProbeSetupState.MOTION_REACHED
    state.alignment_motion_state = ProbeSetupState.MOTION_REACHED
    state.surface_alignment_approved = True
    state.pathing_point_names = ["Clear housing", "Above bearing"]
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert dialog.pathing_point_count_label.text() == "Pathing points: 2"
    dialog.add_pathing_point_button.click()
    popup = dialog.path_dialog
    assert popup.isVisible()
    popup.points.setCurrentRow(1)
    assert popup.up_button.isEnabled()
    assert not popup.down_button.isEnabled()
    popup.up_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_REORDER_PATHING_POINT
    assert ui.requests[-1].pathing_point_index == 1
    assert ui.requests[-1].pathing_point_direction == -1
    state.pathing_point_names = ["Above bearing", "Clear housing"]
    controls.apply_setup_state(state)
    assert popup.points.currentRow() == 0
    popup.move_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_PATHING_POINT
    assert motions[-1].pathing_point_index == 0
    controls.apply_setup_state(state)
    popup.name_field.setText("Third point")
    popup.tolerance_field.setValue(.035)
    popup.add_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_ADD_PATHING_POINT
    assert ui.requests[-1].pathing_point_name == "Third point"
    assert ui.requests[-1].position_tolerance_m == .035
    controls.apply_setup_state(state)
    dialog.move_path_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_PRE_APPROACH_PATH
    popup.close_button.click()
    assert not popup.isVisible()
    dialog.hide()


def test_path_popup_disables_edits_during_movement(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = ProbeSetupState.MOTION_REACHED
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    state.pathing_point_names = ["Point"]
    dialog.path_dialog.name_field.setText("Next")
    dialog.refresh_path_controls(True)
    assert dialog.path_dialog.add_button.isEnabled()
    dialog.refresh_path_controls(False)
    assert not dialog.path_dialog.add_button.isEnabled()
    assert not dialog.path_dialog.move_button.isEnabled()
    assert not dialog.move_path_button.isEnabled()
    dialog.hide()


def test_delete_selection_and_duplicate_name_feedback(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.pathing_point_names = ["Clear housing"]
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    popup = dialog.path_dialog
    popup.name_field.setText(" CLEAR HOUSING ")
    assert not popup.add_button.isEnabled()
    assert "already exists" in popup.name_error_label.text()
    popup.points.setCurrentRow(-1)
    assert not popup.delete_button.isEnabled()
    popup.points.setCurrentRow(0)
    assert popup.delete_button.isEnabled()
    popup.delete_button.click()
    intent = controls.ui.requests[-1]
    assert intent.operation == ProbeSetupIntent.OPERATION_DELETE_PATHING_POINT
    assert intent.pathing_point_index == 0
    assert not popup.delete_button.isEnabled()
    state.pathing_point_names = []
    controls.apply_setup_state(state)
    assert dialog.pathing_point_count_label.text() == "Pathing points: 0"
    assert popup.add_button.isEnabled()
    assert not popup.delete_button.isEnabled()
    assert not popup.name_error_label.text()
    dialog.hide()


def test_final_tolerance_is_only_shown_in_alignment_and_sent_on_save(application):
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show()
    dialog.workflow_stack.setCurrentIndex(dialog.REFERENCE_PAGE)
    assert not controls.probe_position_tolerance_field.isVisible()
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert controls.probe_position_tolerance_field.isVisible()
    controls.probe_position_tolerance_field.setText("0.035")
    controls.handle_use_current_alignment()
    assert controls.ui.requests[-1].position_tolerance_m == .035
    dialog.hide()
