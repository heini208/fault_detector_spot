"""Saved inspection actions can queue without losing request correlation."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_msgs.msg import ApplicationCommandState, OperationalIntent
from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI
from fault_detector_spot.ui.inspection.finalizing_controls import FinalizingInspectionControls
from fault_detector_spot.ui.ros.application_client import ApplicationClient
from test_probe_point_guided_workflow import FakeUI, application, make_state


def controls_with_points():
    ui = FakeUI()
    ui.execute_operation = Mock(return_value="submitted")
    controls = FinalizingInspectionControls(ui)
    state = make_state(with_references=False)
    state.has_base_position = True
    state.probe_point_ids = ["surface", "custom"]
    state.probe_point_fully_custom = [False, True]
    state.probe_point_target_surface_distances_m = [.03, 0.0]
    controls.apply_setup_state(state)
    controls.saved_probe_controls.points_list.setCurrentRow(0)
    return controls, state


def test_all_saved_actions_can_queue_and_keep_their_selection(application):
    controls, state = controls_with_points()
    assert controls.handle_move_to_base_position()
    assert controls.handle_move_to_routine_arm_pose()
    assert controls.saved_probe_controls.handle_motion(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH
    )
    controls.saved_probe_controls.target_distance.setValue(.02)
    assert controls.saved_probe_controls.handle_motion(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    )
    controls.saved_probe_controls.record_duration.setValue(3.5)
    controls.saved_probe_controls.record_retries.setValue(2)
    assert controls.saved_probe_controls.handle_motion(
        OperationalIntent.INTENT_EXECUTE_PROBE_POINT
    )
    controls.saved_probe_controls.points_list.setCurrentRow(1)
    assert controls.saved_probe_controls.handle_motion(
        OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH
    )
    assert controls.handle_move_to_routine_arm_pose()

    calls = controls.ui.execute_operation.call_args_list
    assert [call.args[0].intent for call in calls] == [
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION,
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH,
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH,
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE,
        OperationalIntent.INTENT_EXECUTE_PROBE_POINT,
        OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH,
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH,
    ]
    assert len({call.kwargs["context_id"] for call in calls}) == len(calls)
    assert [call.args[0].probe_point_id for call in calls[2:6]] == [
        "surface", "surface", "surface", "custom"
    ]
    assert calls[3].args[0].target_surface_distance_m == .02
    assert calls[4].args[0].duration_sec == 3.5
    assert calls[4].args[0].retries == 2
    assert not controls.saved_probe_controls.delete_button.isEnabled()

    for call in reversed(calls):
        status = ApplicationCommandState()
        status.context_id = call.kwargs["context_id"]
        status.state = status.STATE_CANCELLED
        controls.handle_application_state(status)
    assert controls.saved_probe_controls.delete_button.isEnabled()
    assert not controls.saved_probe_controls._operation_contexts
    assert not controls._routine_arm_pose_operation_contexts


@pytest.mark.parametrize("field", ["refinement_active", "motion_pending"])
def test_saved_queue_actions_remain_blocked_during_setup_motion(application, field):
    controls, state = controls_with_points()
    setattr(state, field, True)
    controls.apply_setup_state(state)
    assert not controls.move_to_routine_arm_pose_button.isEnabled()
    assert not any(button.isEnabled() for button in controls.saved_probe_controls.action_buttons.values())
    assert not controls.saved_probe_controls.custom_probe_button.isEnabled()


@pytest.mark.parametrize("failure", ["rejected", "goal_exception", "result_exception", "result_request_exception"])
def test_only_explicit_rejection_releases_request_after_transport_error(
    application, monkeypatch, failure
):
    controls, _state = controls_with_points()
    controls.handle_move_to_base_position()
    controls.handle_move_to_routine_arm_pose()
    controls.saved_probe_controls.handle_motion(OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH)
    first_context = controls.ui.execute_operation.call_args.kwargs["context_id"]
    controls.saved_probe_controls.handle_motion(OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE)
    second_context = controls.ui.execute_operation.call_args.kwargs["context_id"]
    goal_response = Future()
    result_response = Future()
    transport = Mock()
    transport.send_goal_async.return_value = goal_response
    monkeypatch.setattr(
        "fault_detector_spot.ui.ros.application_client.ActionClient",
        Mock(return_value=transport),
    )
    client = ApplicationClient(Mock(), "ui")
    harness = SimpleNamespace(inspection_controls=controls, status_label=controls.ui.status_label)
    client.operation_rejected.connect(
        lambda context, detail: Fault_Detector_UI._process_operation_rejected(
            harness, context, detail
        )
    )
    errors = []
    client.request_rejected.connect(errors.append)
    uncertainties = []
    client.operation_uncertain.connect(lambda context, detail: uncertainties.append((context, detail)))
    client.operation_uncertain.connect(
        lambda context, detail: Fault_Detector_UI._process_operation_uncertain(harness, context, detail)
    )
    client.state_changed.connect(controls.handle_application_state)
    assert client.execute(OperationalIntent(), second_context)
    if failure == "goal_exception":
        goal_response.set_exception(RuntimeError("Disconnected"))
    else:
        handle = Mock(accepted=failure != "rejected")
        handle.get_result_async.return_value = result_response
        if failure == "result_request_exception":
            handle.get_result_async.side_effect = RuntimeError("Disconnected")
        goal_response.set_result(handle)
        if failure == "result_exception":
            result_response.set_exception(RuntimeError("Disconnected"))

    if failure == "rejected":
        assert len(errors) == 1
        assert uncertainties == []
        assert list(controls.saved_probe_controls._operation_contexts) == [first_context]
    else:
        assert errors == []
        assert uncertainties == [(second_context, "Disconnected")]
        assert list(controls.saved_probe_controls._operation_contexts) == [first_context, second_context]
        assert "outcome unknown" in harness.status_label.text()
        assert "Outcome unknown" in controls.saved_probe_controls.status_label.text()
        client._receive_state(ApplicationCommandState(
            context_id=second_context, state=ApplicationCommandState.STATE_STOP_CONFIRMED,
        ))
        assert list(controls.saved_probe_controls._operation_contexts) == [first_context]
        assert "stop confirmed" in controls.saved_probe_controls.status_label.text()
    assert controls._routine_arm_pose_operation_contexts
    assert controls._base_position_operation_context
    assert not controls.saved_probe_controls.delete_button.isEnabled()
    assert controls.saved_probe_controls.action_buttons[
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    ].isEnabled()


@pytest.mark.parametrize("state_code", [
    ApplicationCommandState.STATE_UNKNOWN,
    ApplicationCommandState.STATE_STOP_REQUESTED,
    ApplicationCommandState.STATE_STOP_UNCONFIRMED,
])
def test_saved_point_stays_locked_until_stop_is_confirmed(application, state_code):
    controls, _state = controls_with_points()
    controls.saved_probe_controls.handle_motion(OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH)
    context = controls.ui.execute_operation.call_args.kwargs["context_id"]

    controls.handle_application_state(ApplicationCommandState(context_id=context, state=state_code))

    assert context in controls.saved_probe_controls._operation_contexts
    assert not controls.saved_probe_controls.delete_button.isEnabled()
    assert "Movement completed" not in controls.saved_probe_controls.status_label.text()
    controls.handle_application_state(ApplicationCommandState(
        context_id=context, state=ApplicationCommandState.STATE_STOP_CONFIRMED,
    ))
    assert not controls.saved_probe_controls._operation_contexts
    assert controls.saved_probe_controls.delete_button.isEnabled()
    assert "stop confirmed" in controls.saved_probe_controls.status_label.text()
