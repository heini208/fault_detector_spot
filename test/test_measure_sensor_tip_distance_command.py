"""Command transport and setup UI contracts for sensor-tip distance tests."""

from types import SimpleNamespace

import pytest
from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)
from fault_detector_spot.manipulation.commands.measure_sensor_tip_distance_command import MeasureSensorTipDistanceCommand
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import MoveCloseToSurfaceCommand
from fault_detector_spot.inspection.setup.probe_refinement_session import RefinementMotionState, RefinementStage
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls


def intent(expected=0.25):
    message = OperationalIntent()
    message.intent = OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE
    message.aligned_preapproach_distance_m = expected
    return message


def command_subscriber():
    subscriber = CommandSubscriber()
    subscriber._create_command_stamp = lambda: object()
    return subscriber


@pytest.mark.parametrize("expected", [0.12, 0.25, 0.42])
def test_measurement_range_survives_transport_and_reaches_bt_command(expected):
    command = semantic_command_from_message(semantic_command_to_message(
        operational_intent_to_command(intent(expected)),
    ))
    assert command.command_id is CommandID.MEASURE_SENSOR_TIP_DISTANCE
    subscriber = command_subscriber()
    execution = CommandSubscriber._measure_sensor_tip_distance(subscriber, command)
    assert execution.expected_distance_m == pytest.approx(expected)


def test_unspecified_distance_stays_unknown_instead_of_inventing_a_default():
    subscriber = command_subscriber()
    command = operational_intent_to_command(intent(0.0))
    assert CommandSubscriber._measure_sensor_tip_distance(
        subscriber, command,
    ).expected_distance_m is None
    assert MeasureSensorTipDistanceCommand("measure", object()).expected_distance_m is None
    assert MoveCloseToSurfaceCommand("close", object(), .03).expected_distance_m is None


@pytest.mark.parametrize("expected", [-0.1, float("nan"), float("inf")])
def test_invalid_nominal_target_range_is_rejected(expected):
    with pytest.raises(ValueError):
        operational_intent_to_command(intent(expected))
    with pytest.raises(ValueError):
        MeasureSensorTipDistanceCommand("measure", object(), expected)
    with pytest.raises(ValueError):
        MoveCloseToSurfaceCommand("close", object(), .03, expected_distance_m=expected)


def test_close_surface_uses_same_observation_range_from_setup():
    message = intent(.36)
    message.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
    message.target_surface_distance_m = .015
    command = operational_intent_to_command(message)
    subscriber = command_subscriber()
    execution = CommandSubscriber._move_close_to_surface(subscriber, command)
    assert execution.expected_distance_m == pytest.approx(.36)
    assert execution.target_surface_distance_m == pytest.approx(.015)


class Label:
    def __init__(self):
        self.text = ""

    def setText(self, value):
        self.text = value


def controls_for_measurement():
    controls = object.__new__(FinalizingInspectionControls)
    presentation = SimpleNamespace(
        active_stage=RefinementStage.ALIGNMENT,
        pending_motion=None,
        recovery_required=False,
        motion_states={RefinementStage.ALIGNMENT: RefinementMotionState.REACHED},
        aligned_preapproach_distance_m=.25,
    )
    controls._require_refinement_presentation = lambda: presentation
    controls._probe_setup_state = SimpleNamespace(context_id="setup")
    controls._sensor_tip_distance_test_active = False
    controls._sensor_tip_distance_request_id = ""
    controls._sensor_tip_distance_context_id = ""
    controls._surface_move_succeeded = False
    controls._distance_failure_requires_retraction = False
    controls._refresh_refinement_dialog = lambda: None
    controls.refinement_dialog = SimpleNamespace(sensor_tip_distance_status_label=Label())
    calls = []
    controls.ui = SimpleNamespace(
        execute_operation=lambda command, **kwargs: calls.append((command, kwargs)) or "local-id",
        application_client=SimpleNamespace(client_id="ui"),
    )
    return controls, calls


def feedback(controls, state=ApplicationCommandState.STATE_SUCCEEDED, **overrides):
    values = dict(
        intent=OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE,
        client_id="ui", context_id=controls._sensor_tip_distance_context_id,
        request_id="server-request", state=state,
        detail="Sensor tip distance: 0.261 m; returned to initial pose",
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_distance_test_has_unique_context_and_never_submits_close_movement():
    controls, calls = controls_for_measurement()
    assert controls.handle_test_sensor_tip_distance()
    command, kwargs = calls[0]
    assert command.intent == OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE
    assert kwargs["context_id"].startswith("setup/measure-sensor-tip-distance/")
    assert command.aligned_preapproach_distance_m == pytest.approx(.25)
    assert not controls.handle_test_sensor_tip_distance()
    assert len(calls) == 1


@pytest.mark.parametrize("unrelated", [
    {"client_id": "other-ui"}, {"context_id": "old-test"},
    {"intent": OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE},
])
def test_measurement_ui_ignores_unrelated_feedback(unrelated):
    controls, _ = controls_for_measurement()
    controls.handle_test_sensor_tip_distance()
    controls._handle_sensor_tip_distance_status(feedback(controls, **unrelated))
    assert controls._sensor_tip_distance_test_active
    assert "0.261" not in controls.refinement_dialog.sensor_tip_distance_status_label.text


def test_success_reports_distance_without_marking_final_approach_reached():
    controls, _ = controls_for_measurement()
    controls.handle_test_sensor_tip_distance()
    controls._handle_sensor_tip_distance_status(feedback(controls))
    assert not controls._sensor_tip_distance_test_active
    assert "0.261" in controls.refinement_dialog.sensor_tip_distance_status_label.text
    assert not controls._surface_move_succeeded
    assert not controls._distance_failure_requires_retraction


def test_measurement_cancellation_blocks_further_approach_until_recovery():
    controls, _ = controls_for_measurement()
    controls.handle_test_sensor_tip_distance()
    controls._handle_sensor_tip_distance_status(feedback(
        controls, state=ApplicationCommandState.STATE_CANCELLED,
    ))
    assert not controls._sensor_tip_distance_test_active
    assert controls._distance_failure_requires_retraction


def test_success_from_another_request_cannot_complete_running_test():
    controls, _ = controls_for_measurement()
    controls.handle_test_sensor_tip_distance()
    controls._handle_sensor_tip_distance_status(feedback(
        controls, state=ApplicationCommandState.STATE_RUNNING,
    ))
    controls._handle_sensor_tip_distance_status(feedback(
        controls, request_id="unrelated-request",
    ))
    assert controls._sensor_tip_distance_test_active


def test_measurement_behaviour_delegates_start_poll_and_cancellation():
    from fault_detector_spot.manipulation.behaviours.measure_sensor_tip_distance_behaviour import (
        MeasureSensorTipDistanceBehaviour,
    )
    from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
        MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceOutcome,
    )

    command = MeasureSensorTipDistanceCommand("measure", object())
    events = []
    execution = SimpleNamespace(
        active=True,
        feedback_message="Observing target",
        start=lambda actual: events.append(actual) or MeasureSensorTipDistanceOutcome.RUNNING,
        poll=lambda: MeasureSensorTipDistanceOutcome.SUCCESS,
        cancel=lambda: events.append("cancel"),
    )
    behaviour = MeasureSensorTipDistanceBehaviour(
        surface_source=object(), config=MeasureSensorTipDistanceConfig(),
    )
    behaviour.executor = object()
    behaviour._last_command = lambda: command
    behaviour._execution = execution
    behaviour.initialise()
    assert behaviour.update() is Status.RUNNING
    assert events == [command]
    assert command.execution_feedback_message == "Observing target"
    execution.feedback_message = "Sensor tip distance: 0.261 m"
    assert behaviour.update() is Status.SUCCESS
    assert command.execution_feedback_message == execution.feedback_message
    behaviour.terminate(Status.INVALID)
    assert events[-1] == "cancel"


@pytest.mark.parametrize("state,invalidated", [
    ("DISPATCHED", False), ("RUNNING", False), ("SUCCEEDED", False),
    ("FAILED", True), ("CANCELLED", True),
])
def test_scoped_measurement_preserves_alignment_only_until_failure(state, invalidated):
    from threading import RLock
    from fault_detector_spot.application.controllers.command_controller import CommandControllerState
    from fault_detector_spot.application.coordinators.probe_setup_coordinator import ProbeSetupCoordinator

    context = SimpleNamespace(context_id="setup")
    draft = SimpleNamespace(
        context=context, refinement=SimpleNamespace(pending_motion=None),
    )
    calls = []
    coordinator = object.__new__(ProbeSetupCoordinator)
    coordinator._lock = RLock()
    coordinator._drafts = {"setup": draft}
    coordinator._context_lock = lambda _: RLock()
    coordinator._state_listeners = []
    coordinator._advance = lambda actual: actual
    coordinator.refinement_controller = SimpleNamespace(
        invalidate_external_motion=lambda *args: calls.append(args) or True,
    )
    status = SimpleNamespace(
        state=getattr(CommandControllerState, state),
        request=SimpleNamespace(
            context_id="setup/measure-sensor-tip-distance/unique",
            command=SimpleNamespace(command_id=CommandID.MEASURE_SENSOR_TIP_DISTANCE),
        ),
    )
    coordinator._handle_external_command_status(status)
    assert bool(calls) is invalidated


def test_alignment_button_disables_motion_controls_while_measuring():
    from PyQt5.QtWidgets import QApplication
    from test_probe_point_guided_workflow import FakeUI
    from test_probe_safe_approach_navigation import safe_state

    application = QApplication.instance() or QApplication([])
    assert application is not None
    ui = FakeUI()
    ui.execute_operation = lambda *args, **kwargs: "local-id"
    controls = FinalizingInspectionControls(ui)
    state = safe_state()
    state.context_id = "setup"
    state.refinement_stage = state.REFINEMENT_STAGE_ALIGNMENT
    state.safe_approach_motion_state = state.MOTION_REACHED
    state.alignment_motion_state = state.MOTION_REACHED
    controls.apply_setup_state(state)
    controls.refinement_dialog.show_stage(RefinementStage.ALIGNMENT)
    controls._refresh_refinement_dialog()
    dialog = controls.refinement_dialog
    assert dialog.test_distance_measure_button.text() == "Test distance measure"
    assert dialog.test_distance_measure_button.isEnabled()
    assert "0 mm" in dialog.contact_mode_hint_label.text()
    assert controls.handle_test_sensor_tip_distance()
    assert not dialog.test_distance_measure_button.isEnabled()
    assert not controls.move_aligned_pose_button.isEnabled()
    assert not dialog.next_button.isEnabled()
    assert not dialog.close_button.isEnabled()
    assert dialog.emergency_stop_button.isEnabled()
    controls._handle_sensor_tip_distance_status(feedback(controls))
    assert dialog.test_distance_measure_button.isEnabled()
    assert "0.261" in dialog.sensor_tip_distance_status_label.text()
    controls._refresh_refinement_dialog()
    assert "0.261" in dialog.sensor_tip_distance_status_label.text()
    dialog.hide()


def test_lost_result_releases_ui_and_requires_recovery_after_acceptance():
    controls, _ = controls_for_measurement()
    controls.handle_test_sensor_tip_distance()
    controls._handle_sensor_tip_distance_status(feedback(
        controls, state=ApplicationCommandState.STATE_RUNNING,
    ))
    controls.handle_sensor_tip_distance_rejected(
        "Connection lost", controls._sensor_tip_distance_context_id,
    )
    assert not controls._sensor_tip_distance_test_active
    assert controls._distance_failure_requires_retraction
    assert "Connection lost" in controls.refinement_dialog.sensor_tip_distance_status_label.text
