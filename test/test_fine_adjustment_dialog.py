"""Shared adjustment window, target isolation, and speed transport."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

from fault_detector_msgs.msg import ProbeSetupMotionIntent
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementStage, RefinementMotionState
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind, ProbeMotionRequest
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeSetupMotionCommandFactory
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.manipulation.arm_motion_speed import ArmMotionSpeed, ArmMotionSpeedPolicy
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
from fault_detector_spot.application.ros.semantic_command_adapter import semantic_command_to_message, semantic_command_from_message
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.shared.geometry.models import Vector3Data
from test_probe_point_guided_workflow import application, FakeUI
from test_probe_safe_approach_navigation import safe_state
from test_pre_approach_path import begin_setup, snapshot
from test_probe_setup_coordinator import coordinator
from test_command_request_correlation import FakeClock


def test_same_window_has_all_actions_and_keeps_settings(application):
    ui = FakeUI()
    intents = []
    ui.probe_setup_client.execute_motion = lambda intent: intents.append(intent) or "motion"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    main = controls.refinement_dialog
    main.show_stage(RefinementStage.ALIGNMENT)
    controls.open_fine_adjustment_button.click()
    shared = main.fine_adjustment_dialog
    assert shared.isVisible()
    assert controls.refine_frame_dropdown.parent() is shared
    assert len(controls.refinement_buttons['alignment']) == 10
    shared.speed_field.setValue(25.)
    controls.refine_translation_step_field.setText('0.02')
    controls.refinement_buttons['alignment']['pitch_up'].click()
    assert intents[-1].operation == ProbeSetupMotionIntent.OPERATION_ADJUST_ALIGNED_PREAPPROACH
    assert intents[-1].pitch_rad < 0
    assert intents[-1].arm_speed_scale == .25
    controls.apply_setup_state(state)
    shared.hide()
    main.path_dialog.adjust_button.click()
    assert shared.isVisible() and shared.pathing
    assert shared.speed_field.value() == 25.
    controls.refinement_buttons['alignment']['front'].click()
    intent = intents[-1]
    assert intent.operation == intent.OPERATION_ADJUST_PATHING_POSE
    assert intent.translation.x == .02
    assert intent.arm_speed_scale == .25
    assert not controls.refinement_buttons['alignment']['front'].isEnabled()
    main.hide()
    assert not shared.isVisible()


def test_path_adjustment_preserves_final_pose_and_approval(tmp_path):
    probe, commands = coordinator(tmp_path)
    state = begin_setup(probe, commands, approved=True)
    before = deepcopy(state.refinement.candidate_pose(RefinementStage.ALIGNMENT))
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        ProbeMotionKind.ADJUST_PATHING_POSE, translation=Vector3Data(.02, 0., 0.), arm_speed_scale=.25,
    ))
    assert operation.request.command.arm_speed_scale == .25
    probe.submit_motion(operation)
    commands.succeed(operation.request)
    result = snapshot(probe, state).refinement
    assert result.candidate_pose(RefinementStage.ALIGNMENT) == before
    assert result.stage_is_approved(RefinementStage.ALIGNMENT)
    assert result.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.NOT_TESTED


def test_speed_reaches_executor_through_ros_and_bt():
    intent = ProbeSetupMotionIntent()
    intent.operation = intent.OPERATION_ADJUST_PATHING_POSE
    intent.frame = intent.FRAME_HAND
    intent.translation.x = .01
    intent.arm_speed_scale = .25
    intent.position_tolerance_m = .01
    intent.orientation_tolerance_rad = .1
    request = ProbeSetupMotionApi._motion_request(intent)
    assert request.arm_speed_scale == .25
    from dataclasses import replace
    command = replace(ProbeSetupMotionCommandFactory().relative(
        'hand', request.translation, 0., 0., 'probe'), arm_speed_scale=request.arm_speed_scale)
    command = semantic_command_from_message(semantic_command_to_message(command))
    subscriber = CommandSubscriber()
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())
    internal = subscriber.fire_command_sequence(command)[0]
    executor = object.__new__(ArmMovementExecutor)
    executor.speed_policy = ArmMotionSpeedPolicy(default_speed=ArmMotionSpeed(.1, .4))
    executor.probe_motion_planner = Mock(relative_command_is_noop=Mock(return_value=False))
    executor.guarded_probe = Mock()
    executor.relative(internal)
    speed = executor.guarded_probe.call_args.kwargs['speed']
    assert speed.linear_speed_mps == pytest.approx(.025)
    assert speed.angular_speed_rad_s == pytest.approx(.1)


@pytest.mark.parametrize('scale', [0., -1., 1.1, float('nan')])
def test_invalid_speed_is_rejected(scale):
    with pytest.raises(ValueError, match='speed scale'):
        ProbeMotionRequest(ProbeMotionKind.ADJUST_PATHING_POSE, arm_speed_scale=scale).validate()
