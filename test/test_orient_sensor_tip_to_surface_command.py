"""Measured tip orientation travels through setup motion and command transport."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import ProbeSetupMotionIntent
from py_trees.common import Status

from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementMotionState, RefinementStage
from fault_detector_spot.inspection.setup.probe_setup_motion import (
    ProbeMotionKind, ProbeMotionRequest, ProbeSetupMotionCommandFactory,
)
from fault_detector_spot.manipulation.commands.orient_sensor_tip_to_surface_command import OrientSensorTipToSurfaceCommand


@pytest.mark.parametrize("expected", [.12, .25, .42])
def test_setup_orientation_range_and_attachment_survive_transport(expected):
    command = ProbeSetupMotionCommandFactory().orient_sensor_tip_to_surface(
        "lowered_tip", expected,
    )
    command = semantic_command_from_message(semantic_command_to_message(command))
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: Time()
    execution, = subscriber.fire_command_sequence(command)
    assert isinstance(execution, OrientSensorTipToSurfaceCommand)
    assert execution.command_id is CommandID.ORIENT_SENSOR_TIP_TO_SURFACE
    assert execution.motion_sensor_id == "lowered_tip"
    assert execution.expected_distance_m == pytest.approx(expected)


def test_measured_orientation_always_checks_environment_collisions():
    command = ProbeSetupMotionCommandFactory().orient_sensor_tip_to_surface(
        "lowered_tip", .25,
    )
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: Time()
    execution, = subscriber.fire_command_sequence(replace(
        command, ignore_environment_collisions=True,
    ))
    assert execution.ignore_environment_collisions is False


@pytest.mark.parametrize("expected", [0.0, -.1, float("nan"), float("inf")])
def test_invalid_expected_distance_is_rejected(expected):
    with pytest.raises(ValueError, match="must be positive"):
        OrientSensorTipToSurfaceCommand("orient", Time(), "tip", expected)


def test_missing_attachment_is_rejected():
    with pytest.raises(ValueError, match="active sensor geometry"):
        OrientSensorTipToSurfaceCommand("orient", Time(), " ")


def test_setup_motion_action_maps_orientation_both_directions():
    intent = ProbeSetupMotionIntent()
    intent.operation = intent.OPERATION_ORIENT_SENSOR_TIP_TO_SURFACE
    intent.frame = intent.FRAME_SENSOR
    intent.position_tolerance_m = .01
    intent.orientation_tolerance_rad = .05
    motion = ProbeSetupMotionApi._motion_request(intent)
    assert motion.kind is ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE
    assert ProbeSetupMotionApi._motion_operation(motion.kind) == intent.operation


def _reached_setup(tmp_path, *, alignment_reached=True):
    from test_probe_setup_coordinator import (
        coordinator, create_selected_routine, pose,
    )
    from fault_detector_spot.inspection.model.models import ImagePoint

    probe, controller = coordinator(tmp_path)
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    state = probe.select_reference_pixel(
        state.context, "slot1_hand", ImagePoint(u=20, v=30),
        "surface_fit", .10, .20,
    )
    probe.motion_state_source.pose = pose(.8)
    state = probe.begin_refinement(state.context)
    draft = probe._drafts[state.context.context_id]
    refinement = draft.refinement
    refinement.motion_states[RefinementStage.SAFE_APPROACH] = RefinementMotionState.REACHED
    if alignment_reached:
        refinement.motion_states[RefinementStage.ALIGNMENT] = RefinementMotionState.REACHED
        refinement.alignment_candidate_reached = True
        refinement.approve(RefinementStage.ALIGNMENT, probe.motion_state_source.pose)
    probe.refinement_controller.motion_command_factory.orient_sensor_tip_to_surface = (
        ProbeSetupMotionCommandFactory().orient_sensor_tip_to_surface
    )
    return probe, controller, state, draft


def test_setup_orientation_captures_result_and_requires_fresh_approval(tmp_path):
    from test_probe_setup_coordinator import pose, yaw_quaternion

    probe, controller, state, draft = _reached_setup(tmp_path)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        kind=ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE,
    ))
    command = operation.request.command
    assert command.command_id is CommandID.ORIENT_SENSOR_TIP_TO_SURFACE
    assert command.aligned_preapproach_distance_m == pytest.approx(.20)
    assert not command.ignore_environment_collisions
    probe.submit_motion(operation)
    achieved = pose(.803)
    achieved.orientation = yaw_quaternion(8.0)
    probe.motion_state_source.pose = achieved
    controller.succeed(operation.request)
    refinement = draft.refinement
    assert refinement.pending_motion is None
    assert refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED
    assert refinement.candidate_pose(RefinementStage.ALIGNMENT).orientation == achieved.orientation
    assert refinement.candidate_pose(RefinementStage.ALIGNMENT).position == achieved.position
    assert refinement.alignment_orientation_established
    assert not refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    assert refinement.motion_states[RefinementStage.PROBE] is RefinementMotionState.NOT_TESTED


def test_setup_rejects_orientation_before_reaching_safe_pose(tmp_path):
    probe, _, state, draft = _reached_setup(tmp_path, alignment_reached=False)
    draft.refinement.motion_states[RefinementStage.SAFE_APPROACH] = RefinementMotionState.NOT_TESTED
    with pytest.raises(RuntimeError, match="Reach the safe approach"):
        probe.prepare_motion(state.context, ProbeMotionRequest(
            kind=ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE,
        ))
    assert draft.refinement.pending_motion is None


def test_orientation_establishes_unsaved_alignment_and_enables_save(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from test_probe_point_guided_workflow import FakeUI
    from test_probe_safe_approach_navigation import safe_state
    from test_probe_setup_coordinator import pose, yaw_quaternion
    from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
    from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls

    application = QApplication.instance() or QApplication([])
    assert application is not None
    probe, controller, state, draft = _reached_setup(tmp_path, alignment_reached=False)
    assert draft.refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.NOT_TESTED
    assert not draft.refinement.stage_is_approved(RefinementStage.ALIGNMENT)
    operation = probe.prepare_motion(state.context, ProbeMotionRequest(
        kind=ProbeMotionKind.ORIENT_SENSOR_TIP_TO_SURFACE,
    ))
    probe.submit_motion(operation)
    achieved = pose(.723, .12, -.05)
    achieved.orientation = yaw_quaternion(8.0)
    probe.motion_state_source.pose = achieved
    controller.succeed(operation.request)
    completed = probe.snapshot(probe.context(state.context.context_id, "probe-ui"))
    refinement = completed.refinement
    assert refinement.candidate_pose(RefinementStage.ALIGNMENT) == achieved
    assert refinement.motion_states[RefinementStage.ALIGNMENT] is RefinementMotionState.REACHED
    assert refinement.alignment_orientation_established
    assert not refinement.stage_is_approved(RefinementStage.ALIGNMENT)

    message = safe_state()
    ProbeSetupStateAdapter._write_refinement(message, completed)
    controls = FinalizingInspectionControls(FakeUI())
    controls.apply_setup_state(message)
    controls.refinement_dialog.show_stage(RefinementStage.ALIGNMENT)
    controls._refresh_refinement_dialog()
    assert controls.use_current_alignment_button.text() == "Save Final Aligned Pre-approach Pose"
    assert controls.use_current_alignment_button.isEnabled()
    assert not controls.refinement_dialog.next_button.isEnabled()
    controls.refinement_dialog.hide()


def test_orientation_behaviour_owns_execution_and_propagates_cancellation():
    from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementOutcome, ArmMovementUpdate
    from fault_detector_spot.manipulation.behaviours.orient_sensor_tip_to_surface_behaviour import OrientSensorTipToSurfaceBehaviour
    from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import MeasureSensorTipDistanceConfig

    command = OrientSensorTipToSurfaceCommand("orient", Time(), "tip")
    events = []
    execution = SimpleNamespace(
        active=True,
        start=lambda actual: events.append(actual) or ArmMovementUpdate(ArmMovementOutcome.RUNNING, "Measuring target"),
        poll=lambda: ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "Sensor tip oriented to surface"),
        cancel=lambda: events.append("cancel"),
    )
    behaviour = OrientSensorTipToSurfaceBehaviour(
        surface_source=object(), config=MeasureSensorTipDistanceConfig(),
    )
    behaviour.executor = SimpleNamespace(cancel=lambda: pytest.fail("Cancelled arm directly"))
    behaviour._last_command = lambda: command
    behaviour._execution = execution
    behaviour.initialise()
    assert behaviour.update() is Status.RUNNING
    assert events == [command]
    assert command.execution_feedback_message == "Measuring target"
    assert behaviour.update() is Status.SUCCESS
    assert command.execution_feedback_message == "Sensor tip oriented to surface"
    behaviour.terminate(Status.INVALID)
    assert events[-1] == "cancel"


def test_orientation_button_uses_setup_motion_and_shares_measurement_lock():
    from PyQt5.QtWidgets import QApplication
    from test_probe_point_guided_workflow import FakeUI
    from test_probe_safe_approach_navigation import safe_state
    from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls

    application = QApplication.instance() or QApplication([])
    assert application is not None
    controls = FinalizingInspectionControls(FakeUI())
    state = safe_state()
    state.context_id = "setup"
    state.refinement_stage = state.REFINEMENT_STAGE_ALIGNMENT
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_NOT_TESTED
    state.surface_alignment_approved = False
    controls.apply_setup_state(state)
    controls.refinement_dialog.show_stage(RefinementStage.ALIGNMENT)
    controls._refresh_refinement_dialog()
    button = controls.refinement_dialog.orient_sensor_tip_to_surface_button
    assert button.text() == "Orient sensor tip to surface"
    assert button.isEnabled()
    assert controls.orient_to_surface_button.text() == "Orient camera to surface"
    assert not controls.refinement_dialog.test_distance_measure_button.isEnabled()
    calls = []
    controls._submit_probe_motion = lambda intent, label: calls.append((intent, label)) or True
    button.click()
    intent, _ = calls[0]
    assert intent.operation == intent.OPERATION_ORIENT_SENSOR_TIP_TO_SURFACE
    controls._sensor_tip_distance_test_active = True
    controls._refresh_refinement_dialog()
    assert not button.isEnabled()
    controls.refinement_dialog.hide()
