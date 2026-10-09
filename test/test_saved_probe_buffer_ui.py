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
    controls.saved_probe_points_list.setCurrentRow(0)
    return controls, state


def test_all_saved_actions_can_queue_and_keep_their_selection(application):
    controls, state = controls_with_points()
    assert controls.handle_move_to_base_position()
    assert controls.handle_move_to_routine_arm_pose()
    assert controls.handle_saved_probe_motion(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH
    )
    controls.saved_probe_distance.setValue(.02)
    assert controls.handle_saved_probe_motion(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    )
    controls.probe_record_duration.setValue(3.5)
    controls.probe_record_retries.setValue(2)
    assert controls.handle_saved_probe_motion(
        OperationalIntent.INTENT_EXECUTE_PROBE_POINT
    )
    controls.saved_probe_points_list.setCurrentRow(1)
    assert controls.handle_saved_probe_motion(
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
    assert not controls.delete_saved_probe_point_button.isEnabled()

    for call in reversed(calls):
        status = ApplicationCommandState()
        status.context_id = call.kwargs["context_id"]
        status.state = status.STATE_CANCELLED
        controls.handle_application_state(status)
    assert controls.delete_saved_probe_point_button.isEnabled()
    assert not controls._saved_probe_operation_contexts
    assert not controls._routine_arm_pose_operation_contexts


@pytest.mark.parametrize("field", ["refinement_active", "motion_pending"])
def test_saved_queue_actions_remain_blocked_during_setup_motion(application, field):
    controls, state = controls_with_points()
    setattr(state, field, True)
    controls.apply_setup_state(state)
    assert not controls.move_to_routine_arm_pose_button.isEnabled()
    assert not any(button.isEnabled() for button in controls.saved_probe_action_buttons.values())
    assert not controls.saved_custom_probe_button.isEnabled()


@pytest.mark.parametrize("failure", ["rejected", "goal_exception", "result_exception"])
def test_operation_transport_error_releases_only_its_own_request(
    application, monkeypatch, failure
):
    controls, _state = controls_with_points()
    controls.handle_move_to_base_position()
    controls.handle_move_to_routine_arm_pose()
    controls.handle_saved_probe_motion(OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH)
    first_context = controls.ui.execute_operation.call_args.kwargs["context_id"]
    controls.handle_saved_probe_motion(OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE)
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
    harness = SimpleNamespace(inspection_controls=controls)
    client.operation_rejected.connect(
        lambda context, detail: Fault_Detector_UI._process_operation_rejected(
            harness, context, detail
        )
    )
    errors = []
    client.request_rejected.connect(errors.append)
    assert client.execute(OperationalIntent(), second_context)
    if failure == "goal_exception":
        goal_response.set_exception(RuntimeError("Disconnected"))
    else:
        handle = Mock(accepted=failure != "rejected")
        handle.get_result_async.return_value = result_response
        goal_response.set_result(handle)
        if failure == "result_exception":
            result_response.set_exception(RuntimeError("Disconnected"))

    assert len(errors) == 1
    assert list(controls._saved_probe_operation_contexts) == [first_context]
    assert controls._routine_arm_pose_operation_contexts
    assert controls._base_position_operation_context
    assert not controls.delete_saved_probe_point_button.isEnabled()
    assert controls.saved_probe_action_buttons[
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
    ].isEnabled()
