"""Shared custom path editor emits explicit stage and bounded speed intent."""
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupMotionIntent, ProbeSetupState
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from test_probe_point_guided_workflow import application, FakeUI, make_state
from test_probe_safe_approach_navigation import safe_state


def test_custom_choice_skips_reference_capture(application):
    ui = FakeUI()
    controls = FinalizingInspectionControls(ui)
    controls._probe_setup_state = make_state(with_references=False)
    assert controls.handle_start_probe_refinement()
    controls.refinement_dialog.custom_mode_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
    assert ui.requests[-1].fully_custom
    assert ui.capture_calls == []
    assert ui.probe_setup_client.preview_requests == []
    assert controls.refinement_dialog.workflow_stack.currentIndex() == controls.refinement_dialog.SAFE_APPROACH_PAGE
    controls.refinement_dialog.hide()


def test_custom_shared_editor_has_no_surface_controls_and_independent_paths(application):
    ui = FakeUI()
    motions = []
    ui.probe_setup_client.execute_motion = lambda intent: motions.append(intent) or "motion"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.fully_custom = True
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.has_alignment_candidate = False
    state.has_final_probe_candidate = False
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert not dialog.aligned_distance_field.isVisible()
    assert not controls.orient_to_surface_button.isVisible()
    assert not dialog.back_away_surface_button.isVisible()
    assert not controls.move_aligned_pose_button.isEnabled()
    assert controls.use_current_alignment_button.isEnabled()
    assert dialog.pose_comparison_labels[RefinementStage.ALIGNMENT]["candidate"].text() == "Not set"
    assert dialog.candidate_speed_field.value() == 100
    dialog.candidate_speed_field.setValue(65)
    controls.use_current_alignment_button.click()
    assert ui.requests[-1].arm_speed_scale == .65
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_APPROVE_ALIGNED_POSE
    state.surface_alignment_approved = True
    state.has_alignment_candidate = True
    state.alignment_motion_state = state.MOTION_REACHED
    state.final_pathing_point_names = ["Final waypoint"]
    state.pathing_point_names = ["Pre waypoint"]
    controls.apply_setup_state(state)
    dialog.show_stage(RefinementStage.PROBE)
    assert dialog.workflow_stack.currentIndex() == dialog.ALIGNMENT_PAGE
    assert dialog.path_dialog.points.item(0).text() == "Final waypoint"
    assert dialog.candidate_speed_field.value() == 20
    assert not dialog.move_path_button.isEnabled()
    controls.use_current_alignment_button.click()
    assert ui.requests[-1].operation == ProbeSetupIntent.OPERATION_APPROVE_CUSTOM_PROBE
    assert ui.requests[-1].arm_speed_scale == .2
    controls.apply_setup_state(state)
    dialog.fine_adjustment_dialog.open_for(False)
    assert controls.refine_translation_step_field.text() == "0.001"
    assert controls.refine_rotation_step_field.text() == "1.0"
    assert dialog.fine_adjustment_dialog.speed_field.value() == 10
    controls.refine_translation_step_field.setText("0.2")
    assert controls.handle_fine_adjustment("up")
    assert motions[-1].path_stage == "probe"
    assert motions[-1].translation.z == .2
    assert motions[-1].arm_speed_scale == .1
    dialog.hide()


def test_speed_sliders_and_stage_adjustments_remain_independent(application):
    from PyQt5.QtWidgets import QSlider
    controls = FinalizingInspectionControls(FakeUI())
    dialog = controls.refinement_dialog
    fine = dialog.fine_adjustment_dialog
    for field in (fine.speed_field, dialog.candidate_speed_field, dialog.path_dialog.speed_field):
        assert isinstance(field.slider, QSlider)
        field.setValue(45.5)
        assert field.value() == 46
        assert field.label.text() == "46 %"
        field.slider.setValue(47)
        assert field.value() == 47
        assert field.slider.singleStep() == 1
        field.setValue(120)
        assert field.value() == 100
        field.setValue(0)
        assert field.value() == 1
    controls.refine_translation_step_field.setText("0.05")
    fine.speed_field.setValue(70)
    fine.set_stage(True)
    assert controls.refine_translation_step_field.text() == "0.001"
    assert fine.speed_field.value() == 10
    fine.speed_field.setValue(8)
    fine.set_stage(False)
    assert controls.refine_translation_step_field.text() == "0.05"
    assert fine.speed_field.value() == 70
    fine.set_stage(True)
    assert fine.speed_field.value() == 8


def test_final_step_returns_to_previous_aligned_candidate(application):
    ui = FakeUI()
    motions = []
    ui.probe_setup_client.execute_motion = lambda intent: motions.append(intent) or "motion"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.fully_custom = True
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    state.surface_alignment_approved = True
    state.has_alignment_candidate = True
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.PROBE)
    assert dialog.move_safe_pose_button.text() == "Move to Saved Aligned Pre-approach"
    dialog.move_safe_pose_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_ALIGNED_PREAPPROACH
    assert motions[-1].path_stage == "alignment"
    assert controls._refinement_presentation.active_stage is RefinementStage.PROBE
    # Failed returns can be retried without navigating back to the previous step.
    state.alignment_motion_state = state.MOTION_FAILED
    controls.apply_setup_state(state)
    assert dialog.move_safe_pose_button.isEnabled()
    state.motion_pending = True
    controls.apply_setup_state(state)
    assert not dialog.move_safe_pose_button.isEnabled()
    state.motion_pending = False
    controls.apply_setup_state(state)
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert dialog.move_safe_pose_button.text() == "Move to Safe Pre-approach Pose"
    dialog.move_safe_pose_button.click()
    assert motions[-1].operation == ProbeSetupMotionIntent.OPERATION_MOVE_SAFE_APPROACH
    dialog.hide()


def test_custom_alignment_reuses_full_arm_controls_and_closes_on_navigation(application):
    from unittest.mock import Mock
    from fault_detector_msgs.msg import OperationalIntent
    ui = FakeUI()
    ui.update_tags_dropdown = lambda dropdown: None
    ui.update_frames_dropdown = lambda dropdown: dropdown.addItem("body")
    ui.execute_operation = Mock(return_value="operation")
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.fully_custom = True
    state.safe_approach_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert dialog.full_control_button.isVisible()
    assert dialog.full_control_button.isEnabled()
    dialog.full_control_button.click()
    popup = dialog.full_control_dialog
    assert popup.isVisible()
    # The reused movement widget retains the existing operational command route.
    from PyQt5.QtWidgets import QPushButton
    move = next(button for button in popup.findChildren(QPushButton)
                if button.text() == "Move Arm by Offset")
    move.click()
    assert ui.execute_operation.call_args.args[0].intent == OperationalIntent.INTENT_MOVE_ARM_RELATIVE
    dialog.show_stage(RefinementStage.PROBE)
    assert dialog.full_control_dialog is None
    assert not dialog.full_control_button.isVisible()
    dialog.hide()


def test_external_motion_update_disables_next_but_keeps_saved_candidate(application, tmp_path):
    from unittest.mock import Mock
    from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi
    from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
    from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind
    from test_custom_probe_workflow import begin_custom, move, latest
    from test_command_request_correlation import FakeClock
    from fault_detector_spot.application.commanding.command_ids import CommandID
    from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
    from fault_detector_spot.application.commanding.command_request import CommandRequest, CommandOrigin, RecordingPolicy
    from fault_detector_spot.application.controllers.command_controller import CommandControllerState, CommandControllerStatus

    probe, commands, state = begin_custom(tmp_path)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_SAFE_APPROACH)
    state = probe.approve_aligned_pose(state.context)
    api = ProbeSetupApi.__new__(ProbeSetupApi)
    api.state_adapter = ProbeSetupStateAdapter(FakeClock())
    api.state_publisher = Mock()
    probe.add_state_listener(api._publish_external_motion_state)
    controls = FinalizingInspectionControls(FakeUI())
    controls.apply_setup_state(api.state_adapter.message(state, 0, ProbeSetupState.STATE_READY, "Ready"))
    dialog = controls.refinement_dialog
    dialog.show_stage(RefinementStage.ALIGNMENT)
    assert dialog.next_button.isEnabled()
    request = CommandRequest.create(command=SemanticCommand(command_id=CommandID.MOVE_ARM_RELATIVE),
                                    client_id="probe-ui", origin=CommandOrigin.OPERATIONAL,
                                    recording_policy=RecordingPolicy.EXCLUDE)
    for listener in tuple(commands.listeners):
        listener(CommandControllerStatus(request, CommandControllerState.DISPATCHED))
    message = api.state_publisher.publish.call_args.args[0]
    assert message.surface_alignment_approved
    assert message.alignment_motion_state == message.MOTION_NOT_TESTED
    controls.apply_setup_state(message)
    assert not dialog.next_button.isEnabled()
    assert not controls.handle_refinement_next()
    assert controls.move_aligned_pose_button.isEnabled()
    assert controls.use_current_alignment_button.isEnabled()
    state = latest(probe, state)
    state, _ = move(probe, commands, state, ProbeMotionKind.MOVE_ALIGNED_PREAPPROACH,
                    achieved=state.setup.aligned_preapproach_pose_object)
    controls.apply_setup_state(api.state_adapter.message(state, 0, ProbeSetupState.STATE_READY, "Reached"))
    assert dialog.next_button.isEnabled()
    dialog.hide()
